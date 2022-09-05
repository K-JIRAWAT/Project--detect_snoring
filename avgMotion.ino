#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

int count = 0;
double acc_x, acc_y, acc_z, avg_arr[5] = {0, 0, 0, 0, 0};

double avg_acc(double x, double y, double z);
void emptyarr();
void swap(double *xp, double *yp);
void bubbleSort(double arr[], int n);

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(0.5);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
}

void loop() {

  if (mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    acc_x = a.acceleration.x;
    acc_y = a.acceleration.y;
    acc_z = a.acceleration.z;



    //Serial.println(avg_acc(acc_x,acc_y,acc_z));
    if (count < 5) {
      //double data = avg_acc(acc_x, acc_y, acc_z);
      double data = avg_acc(g.gyro.x, g.gyro.y, g.gyro.z);
      Serial.print("Count = ");
      Serial.println(count);
      Serial.println(".............");
      avg_arr[count] = data;
      Serial.print("Count arr = ");
      Serial.println(avg_arr[count]);
      Serial.println(".............");
      count++;
    }
    else if (count == 5) {
      bubbleSort(avg_arr, 5);
      for (int i = 0; i < 5; i++) {
        Serial.println("-------------");
        Serial.println(avg_arr[i]);
        Serial.println("-------------");
      }
      double differ = avg_arr[4] - avg_arr[0];

      if ((differ >= 0.3) && (differ <= 0.8)) {
        Serial.println(differ);
        Serial.println("พบเจอคนกัดฟัน");
        emptyarr();
        delay(2000);
      }
      else if (differ < 0.3) {
        Serial.println(differ);
        Serial.println("ข้าม");
        emptyarr();
        delay(2000);
      }
      else {
        Serial.println(differ);
        Serial.println("นอนกลิ้ง");
        emptyarr();
        delay(2000);
      }
    }
    else {
      return;
    }



  }

  delay(10);
}


double avg_acc(double x, double y, double z) {
  Serial.println(x);
  Serial.println(y);
  Serial.println(z);
  Serial.println("-------------");
  return sqrt(sq(x) + sq(y) + sq(z));
}

void emptyarr() {
  count = 0;
  avg_arr[0] = 0;
  avg_arr[1] = 0;
  avg_arr[2] = 0;
  avg_arr[3] = 0;
  avg_arr[4] = 0;
}



void swap(double *xp, double *yp)
{
  double temp = *xp;
  *xp = *yp;
  *yp = temp;
}

void bubbleSort(double arr[], int n)
{
  int i, j;
  for (i = 0; i < n - 1; i++)

    // Last i elements are already
    // in place
    for (j = 0; j < n - i - 1; j++)
      if (arr[j] > arr[j + 1])
        swap(&arr[j], &arr[j + 1]);
}
