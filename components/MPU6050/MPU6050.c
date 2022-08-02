#include "include/MPU6050.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "math.h"

/* Configuracion I2C */
#define MPU_INT     32      //TODO:implementar
#define MPU_SDA     25
#define MPU_SCL     27
#define MPU_FREQ    400000
#define MPU_ADDR    0x68

#define MPU_ACC_Base    0x3B
#define MPU_TEMP_Base   0x41
#define MPU_GYRO_Base   0x43

#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TX_BUF_DISABLE 0

#define I2C_TAG     "COMUNICACION I2C"
#define MPU_TAG     "MPU6050"

#define RAD_TO_DEG 57.295779513082320876798154814105
#define PI 3.14159265359
#define INTERVAL_FILTER 0.001                        //1ms


TaskHandle_t xHandleMPU= NULL;
static void vTaskMpu(void *pvParameters);


const i2c_port_t  i2c_port = I2C_NUM_0;
uint8_t vDATA[6];
int16_t vACC[3],vGYRO[3];
float vAngles[3],vAnglesAcc[3],vAnglesGyro[3],vAngles_ant[3],vAngles_calib[3];


/* 
*   Configuracion del periferico i2c
*/
void i2c_init(void){

    i2c_config_t config_i2c={
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU_SDA,
        .scl_io_num = MPU_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MPU_FREQ
    };
    
    i2c_param_config(i2c_port,&config_i2c);
    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/*
*   Lectura de i2c 
*   \param: direccion de lectura, longitud
*/
void i2c_read(int8_t readAddr,uint8_t len){

    i2c_cmd_handle_t cmd= i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);   // modo escritura

    i2c_master_write_byte(cmd, readAddr, ACK_CHECK_EN);                             // escribo la direccion a leer
    i2c_master_start(cmd);                                                          // 
        
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);      // modo lectura
    if(len > 1){
        //  i2c_master_read(cmd, data, len - 1, ACK_VAL);                              // leo el dato y sigo leyendo
        i2c_master_read(cmd, vDATA, len - 1, ACK_VAL);                              // leo el dato y sigo leyendo
    }
    i2c_master_read_byte(cmd, vDATA + len - 1, NACK_VAL);                            // leo el dato y es la ultima lectura

 
    i2c_master_stop(cmd);                                                           // detengo comunicacion i2c
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    if(ret != ESP_OK){
        ESP_LOGE(I2C_TAG,"ERROR LECTURA: FUNCION(%s), ERROR: %s\n",__func__, esp_err_to_name(ret));
    }
}

/*
*   Lectura de registro por i2c 
*   \param: direccion de lectura
*/
uint16_t i2c_readReg(int8_t readAddr){

    uint8_t data[2];
    
    i2c_cmd_handle_t cmd= i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);   // modo escritura

    i2c_master_write_byte(cmd, readAddr, ACK_CHECK_EN);                             // escribo la direccion a leer
    i2c_master_start(cmd);                                                          // 
        
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);      // modo lectura

    i2c_master_read(cmd, &data[0], 1 , ACK_VAL);                                     // leo el dato y sigo leyendo
    i2c_master_read_byte(cmd, &data[1], NACK_VAL);                                   // leo el dato y es la ultima lectura
 
    i2c_master_stop(cmd);                                                           // detengo comunicacion i2c
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    if(ret != ESP_OK){
        ESP_LOGE(I2C_TAG,"ERROR LECTURA: FUNCION(%s), ERROR: %s\n",__func__, esp_err_to_name(ret));
    }

    return (data[0]*0xff)|data[1];
}


/*
*   Escritura de i2c
*   \param: writeAddr: direccion a escribir
*   \param: writeVal:  valor a escribir
*   \param: longitud
*/
void i2c_write(int8_t writeAddr,uint8_t writeVal, uint16_t len){

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    if (len) {
        i2c_master_write_byte(cmd, writeAddr, ACK_CHECK_EN);
    }
    for (int i = 0; i < len; i++) {
        i2c_master_write_byte(cmd, writeVal, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
  
    if(ret != ESP_OK){
        ESP_LOGE(I2C_TAG,"ERROR ESCRITURA: FUNCION(%s), ERROR: %s\n",__func__, esp_err_to_name(ret));
    }
}


/*
*   Funcion para inicializar el MPU6050
*   Durante 100ms se realiza la calibracion
*/
void mpu_init(void){
    uint8_t i,j;
    float vSumCalibAngle[3]={0};

    i2c_init();
    printf("i2c inicializado\n");

    printf("power_reg: %d\n",i2c_readReg(0x6B));

    /* power managment 1*/
    i2c_write(0x6B,0x00,1);

    if(i2c_readReg(0x6B)>>8 == 0x00){
        printf("mpu configurado\n");
    }
    else{
        printf("mpu ERROR CONFIG\n");
    }

    xTaskCreate(vTaskMpu,"tarea Mpu",4096,NULL,3,&xHandleMPU);                                  //corregir tamaño de pila y prioridad

    // printf("Calibrando NO MOVER\n");
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // for(i=0;i<10;i++){
    //     for(j=0;j<3;j++){
    //         vSumCalibAngle[j]+=vAngles[j];
    //         printf(".");
    //         vTaskDelay(pdMS_TO_TICKS(10));
    //     }
    // }

    // for(i=0;i<3;i++){
    //     vAngles_calib[i] = vSumCalibAngle[i]/10;
    //     printf("Calib Angle %d: %f\n",i,vAngles_calib[i]);
    // }

    // for(j=0;j<3;j++){
    //     printf("angulo %d resul: %f\n",j,getAngle(j));
    //     }
}

void mpu_deInit(void){
    vTaskDelete(xHandleMPU);
}

/*
*   Funcion para la lectura de cada eje,tanto del ACC como del GYRO
*   \return: uint16_t con el valor del ACC
*/
int16_t mpu_readAxis(uint8_t axis){

    return i2c_readReg(axis);                                   // leo los 6 registros del ACC
}


/*
*   Funcion para la lectura de los 3 ejes del acelerometro y los 3 del giroscopo
*   el resultado se almacena en un vector global llamado vACC y vGYRO
*/
void mpu_readAllAxis(void){
    uint8_t i;

    for(i=0;i<3;i++){
        vACC[i] = mpu_readAxis(MPU_ACC_Base + (i*2));
        vGYRO[i] = mpu_readAxis(MPU_GYRO_Base + (i*2));
    }
}

float getAngle(uint8_t eje){
    return vAngles[eje];//-vAngles_calib[eje]+90;
}

void mpu_fillQueue(void){

//TODO: crear funcion que carga la cola 
}

/*
*   Tarea para calculo de angulos del MPU6050
*   Se ejecuta a intervalos regulares definidos en INTERVAL_FILTER
*/
static void vTaskMpu(void *pvParameters){                   // TODO: agregar cola para pasar tareas a la IMU, ademas se debe ejecutar cada 1ms
    uint8_t i;

    for(;;){

        mpu_readAllAxis();

        //Calcular los ángulos con acelerometro
        vAnglesAcc[0]=atan(vACC[1]/sqrt(pow(vACC[0],2) + pow(vACC[2],2)))*(180.0/PI);
        vAnglesAcc[1]=atan(-vACC[0]/sqrt(pow(vACC[1],2) + pow(vACC[2],2)))*(180.0/PI);                      // OJO SIGNO -
        // vAnglesAcc[2]=0;                                                                                 // falta implementar
        vAnglesAcc[2]=atan(-vACC[2]/sqrt(pow(vACC[0],2) + pow(vACC[1],2)))*(180.0/PI);

        for(i=0;i<3;i++){
        
            //Calcular angulo de rotación con giroscopio y filtro complemento  
            vAngles[i] = (0.98*(vAngles_ant[i]+(vGYRO[i]/131)*INTERVAL_FILTER) + 0.02*vAnglesAcc[i]);
            vAngles_ant[i] = vAngles[i];

        }
        // vAngles[2]=0;                                                                                    // falta implementar
        // vAngles_ant[2]=0;
        vTaskDelay(pdMS_TO_TICKS(INTERVAL_FILTER*1000));                                                    //todo: reemplazar por algo mas preciso
    }
}