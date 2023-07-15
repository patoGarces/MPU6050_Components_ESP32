#include "include/MPU6050.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "math.h"

#include "../../include/main.h"

#define PRIORITY_MPU    5
#define MPU_CORE        1

/* Configuracion I2C */
//#define MPU_INT     31      //TODO:implementar
#define MPU_SDA     32
#define MPU_SCL     33
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

/* Periodo de calculo para el filtro complementario*/
#define INTERVAL_FILTER     PERIOD_IMU_MS/1000


const float MOUNT_OFFSET_AXIS[3]={
    0.00,
    0.00,
    0.00
};

TaskHandle_t xHandleMPU= NULL;
static void vTaskMpu(void *pvParameters);
static void initTimer(void);

static SemaphoreHandle_t semaphoreReadMpu;

QueueSetHandle_t newAnglesQueue;

const i2c_port_t  i2c_port = I2C_NUM_0;
int16_t vACC[3],vGYRO[3];
float vAngles[3],vAngles_calib[3];


/* 
*   Configuracion del periferico i2c
*/
static void i2c_init(void){

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
*   Lectura de i2c de 8 bits
*   \param: direccion de lectura, longitud
*/
static uint8_t i2c_read(int8_t readAddr){
    uint8_t data = 0;

    i2c_cmd_handle_t cmd= i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);   // modo escritura

    i2c_master_write_byte(cmd, readAddr, ACK_CHECK_EN);                             // escribo la direccion a leer
    i2c_master_start(cmd); 
        
    i2c_master_write_byte(cmd, MPU_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);      // modo lectura

    i2c_master_read_byte(cmd, &data, NACK_VAL);                                     // leo el dato y es la ultima lectura

    i2c_master_stop(cmd);                                                           // detengo comunicacion i2c
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK){
        ESP_LOGE(I2C_TAG,"ERROR LECTURA: FUNCION(%s), ERROR: %s\n",__func__, esp_err_to_name(ret));
    }
    return data;
}

/*
*   Lectura de registro por i2c 
*   \param: direccion de lectura
*/
static uint16_t i2c_readReg(int8_t readAddr){

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
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
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
*   \param: len: longitud
*/
static void i2c_write(int8_t writeAddr,uint8_t writeVal, uint16_t len){

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
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
  
    if(ret != ESP_OK){
        ESP_LOGE(I2C_TAG,"ERROR ESCRITURA: FUNCION(%s), ERROR: %s\n",__func__, esp_err_to_name(ret));
    }
}

/*
*   Funcion para inicializar el MPU6050
*   Durante 100ms se realiza la calibracion
*/
esp_err_t mpu_init(void){
    uint8_t i,j;
    float vSumCalibAngle[3]={0.00,0.00,0.00};

    i2c_init();
    printf("i2c inicializado\n");

    /* power managment 1*/
    i2c_write(0x6B,0x00,1);

    // printf("power_reg: %x\n",i2c_read(0x6B));

    if(i2c_readReg(0x6B)>>8 == 0x00){
        printf("mpu configurado\n");
    }
    else{
        printf("mpu ERROR CONFIG\n");
        return ESP_FAIL;
    }

    xTaskCreatePinnedToCore(vTaskMpu,"Task Mpu",4096,NULL,PRIORITY_MPU,&xHandleMPU,MPU_CORE);

    initTimer();

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
    //     printf("Media calculada %d: %f, angulo calib: %f\n",i,vAngles_calib[i],getAngle(i));
    // }

    return ESP_OK;
}

void mpu_deInit(void){
    vTaskDelete(xHandleMPU);
}

static bool readMpuCb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx){

    BaseType_t high_task_awoken = pdFALSE;
    xSemaphoreGiveFromISR(semaphoreReadMpu, &high_task_awoken);
    return (high_task_awoken == pdTRUE);
}

static void initTimer(void){

    gptimer_handle_t handleTimer = NULL;

    gptimer_config_t timerConfig = {

        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig,&handleTimer));
    
    gptimer_alarm_config_t alarmMpu ={
        .alarm_count = PERIOD_IMU_MS*1000,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(handleTimer,&alarmMpu));
    
    gptimer_event_callbacks_t newCallback ={
        .on_alarm = readMpuCb,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(handleTimer,&newCallback,NULL));
    ESP_ERROR_CHECK(gptimer_enable(handleTimer));
    ESP_ERROR_CHECK(gptimer_start(handleTimer));

    printf("timer configurado OK\n");

}

/*
*   Funcion para la lectura de los 3 ejes del acelerometro y los 3 del giroscopo
*   el resultado se almacena en un vector global llamado vACC y vGYRO
*/
void mpu_readAllAxis(void){
    uint8_t i;

    for(i=0;i<3;i++){
        vACC[i] = i2c_readReg(MPU_ACC_Base + (i*2));
        vGYRO[i] = i2c_readReg(MPU_GYRO_Base + (i*2));
    }
}

/*
 * Permite leer el angulo actual asincronicamente
 * Solo para visualizar o debug, para tareas que necesiten consumir los datos sincronicamente se utiliza una cola 
 */
float getAngle(uint8_t eje){
    return vAngles[eje];
}

rawData_t getRawData(){
    rawData_t rawData;

    rawData.accX = vACC[0];
    rawData.accY = vACC[1];
    rawData.accZ = vACC[2];
    rawData.gyX = vGYRO[0];
    rawData.gyY = vGYRO[1];
    rawData.gyZ = vGYRO[2];
    return rawData;
}

/*
*   Tarea para calculo de angulos del MPU6050
*/
static void vTaskMpu(void *pvParameters){                   // TODO: agregar cola para pasar tareas a la IMU
    uint8_t eje;
    float vAnglesAcc[3],vAngles_ant[3];

    semaphoreReadMpu = xSemaphoreCreateBinary();

    newAnglesQueue = xQueueCreate(1,sizeof(vAngles));

    if (semaphoreReadMpu == NULL) {
        printf("No se pudo crear el semaforo\n");
    }

    for(;;){
        if (xSemaphoreTake(semaphoreReadMpu, portMAX_DELAY) == pdPASS) {

            // gpio_set_level(13,1);
            mpu_readAllAxis();

            //Calcular los ángulos con acelerometro
            vAnglesAcc[AXIS_ANGLE_X]  = (atan2(-vACC[1],vACC[2])*180.0)/M_PI;
            vAnglesAcc[AXIS_ANGLE_Y] = (atan2(vACC[0], sqrt(vACC[1]*vACC[1] + vACC[2]*vACC[2]))*180.0)/M_PI;

            for(eje=0;eje<2;eje++){
                vAngles[eje] = (0.98*(vAngles_ant[eje]+(vGYRO[eje]/131)*INTERVAL_FILTER) + 0.02*vAnglesAcc[eje]);

                vAngles_ant[eje] = vAngles[eje];
                vAngles[eje] = vAngles[eje]- vAngles_calib[eje] - MOUNT_OFFSET_AXIS[eje];
            }
            xQueueSend(newAnglesQueue,( void * ) &vAngles, 1);
            // gpio_set_level(13,0);
       }
    }
}