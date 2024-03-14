#include <Arduino.h>
//#include <FreeRTOS.h>

SemaphoreHandle_t mutex;
// put function declarations here:
TaskHandle_t Tarea1handle;
TaskHandle_t Tarea2handle;

const int led1 = 2; //Pin del LED
const int led0 = 37;

void Tarea0(void * parameter){
  for(;;){
    Serial.print("Funciona 0");
    // Turn the LED on
    digitalWrite(led0, HIGH);
    // Pause the task for 500ms
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // Turn the LED off
    digitalWrite(led0, LOW);
    // Pause the task again for 500ms
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void Tarea1(void * parameter){

  /*Serial.print("Tarea 1 is running on");
  Serial.println(xPortGetCoreID());*/
 
  for(;;){ // infinite loop

    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE){

    Serial.print(" Led encendido ");
  
    // Turn the LED on
    digitalWrite(led1, HIGH);

    vTaskDelay(1000);
    // Libero el semaforo 
    xSemaphoreGive(mutex);


    }
    // Pause the task for 500ms
    vTaskDelay(100);


  }
}

void Tarea2(void * parameter){

  /*Serial.print(" Tarea 2 is running on: " );
  Serial.println(xPortGetCoreID());*/

  for(;;){

    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE){


      Serial.print(" Led apagado ");
      // Turn the LED off
      digitalWrite(led1, LOW);

      vTaskDelay(1000);
      xSemaphoreGive(mutex);
    }

      // Pause the task again for 500ms
      vTaskDelay(100);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(led1, OUTPUT);
  pinMode(led0, OUTPUT);

  // Inicializar el semáforo
  mutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore( Tarea1,"Tarea1", 10000,NULL,1,&Tarea1handle,1);
  


  xTaskCreatePinnedToCore( Tarea2,"Tarea2", 10000,NULL,1,&Tarea2handle,0);
  

  xTaskCreatePinnedToCore(
    Tarea0, // Function that should be called
    "Tarea0", // Name of the task (for debugging)
    1000, // Stack size (bytes)
    NULL, // Parameter to pass
    1, // Task priority
    NULL, // Task handle
    1
  );

}

void loop() {
  // put your main code here, to run repeatedly:

}  