#include <Arduino.h>

#include <Wire.h>

#include <TinyGPSplus.h>
#include <SoftwareSerial.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#define GPS_RX 3
#define GPS_TX 2
#define bluetooth_RX 8
#define bluetooth_TX 9
#define GPS_BAUD 9600
#define bluetooth_BAUD 9600
#define WAYPOINT_THRESHHOLD 5
#define LED_pin 13

#define dummy_lat 30
#define dummy_lng -80

#define left_motor_pwm 3
#define left_motor_A 4
#define left_motor_B 5
#define right_motor_C 6
#define right_motor_D 7
#define right_motor_pwm 8
#define waste_pull_motor 11
#define esp_pin 13

#define LEFT_topSpeed 150
#define RIGHT_topSpeed 150

TinyGPSPlus gps;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

SoftwareSerial ss_gps(GPS_RX, GPS_TX);
SoftwareSerial ss_bluetooth(A3, A2);

double lat1, lon1, k_L = LEFT_topSpeed / 180, k_R = RIGHT_topSpeed / 180;
double coordinates_lat[4], coordinates_lon[4];
int no_points = 10;
double points_lat[100];
double points_lon[100];
double dist = 0;
const double toRadian = 0.01745329251;
const double toDegree = 57.2957795131;
bool auto_mode = false;

void compass_init()
{
  if (!mag.begin())
  {
    Serial.println("no HMC5883 detected");
    while (1)
      ;
  }
}

double heading_compass()
{
  sensors_event_t event;
  mag.getEvent(&event);

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  // Serial.print("X: ");
  // Serial.print(event.magnetic.x);
  // Serial.print("  ");
  // Serial.print("Y: ");
  // Serial.print(event.magnetic.y);
  // Serial.print("  ");
  // Serial.print("Z: ");
  // Serial.print(event.magnetic.z);
  // Serial.print("  ");
  // Serial.println("uT");

  double heading = -atan2(event.magnetic.y, event.magnetic.x);
  heading += 0.006;
  heading = heading * 180 / M_PI;
  heading = constrain(heading, -180, 180);
  // Serial.print("Heading: ");
  // Serial.println(heading);
  return heading;
}

void intermediate_point(double lat1, double lon1, double lat2, double lon2, double angular_dist, double frac, int index)
{
  double a = sin((1 - frac) * angular_dist) / sin(angular_dist);
  double b = sin(frac * angular_dist) / sin(angular_dist);
  double x = a * cos(lat1) * cos(lon1) + b * cos(lat2) * cos(lon2);
  double y = a * cos(lat1) * sin(lon1) + b * cos(lat2) * sin(lon2);
  double z = a * sin(lat1) + b * sin(lat2);
  points_lat[index] = atan2(z, sqrt((x * x) + (y * y))) * toDegree;
  points_lon[index] = atan2(y, x) * toDegree;
}

void get_points()
{
  // Theta
  double lat1 = coordinates_lat[0] * toRadian;
  double lat2 = coordinates_lat[1] * toRadian;
  double lat3 = coordinates_lat[2] * toRadian;
  double lat4 = coordinates_lat[3] * toRadian;

  // labmda
  double lon1 = coordinates_lon[0] * toRadian;
  double lon2 = coordinates_lon[1] * toRadian;
  double lon3 = coordinates_lon[2] * toRadian;
  double lon4 = coordinates_lon[3] * toRadian;

  // Delta coordinates
  // double deltaLat= (lat4 - lat1);
  // double deltaLon = (lon4 - lon1);

  // Distance
  double a = sin((lat4 - lat1) / 2) * sin((lat4 - lat1) / 2) + cos(lat1) * cos(lat4) * sin((lon4 - lon1) / 2) * sin((lon4 - lon1) / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  int i = 0, f = 0;
  intermediate_point(lat1, lon1, lat4, lon4, c, (double)0, 0);
  i += 3;
  for (f = 1; i < no_points * 2; i += 3, f++)
  {
    intermediate_point(lat1, lon1, lat4, lon4, c, (double)f / no_points, i);
    f++, i++;
    if (i >= no_points * 2)
      break;
    intermediate_point(lat1, lon1, lat4, lon4, c, (double)f / no_points, i);
  }

  // deltaLat = (lat3 - lat2);
  // deltaLon = (lon3 - lon2);
  a = sin((lat3 - lat2) / 2) * sin((lat3 - lat2) / 2) + cos(lat2) * cos(lat3) * sin((lon3 - lon2) / 2) * sin((lon3 - lon2) / 2);
  c = 2 * atan2(sqrt(a), sqrt(1 - a));
  i = 0, f = 0;
  i++;
  for (; i < no_points * 2 - 2; i += 3, f++)
  {
    intermediate_point(lat2, lon2, lat3, lon3, c, (double)f / no_points, i);
    f++, i++;
    intermediate_point(lat2, lon2, lat3, lon3, c, (double)f / no_points, i);
  }
  ss_bluetooth.listen();
  for (int i = 0; i < no_points * 2; i++)
  {
    ss_bluetooth.print(points_lat[i], 6);
    ss_bluetooth.print(",");
    ss_bluetooth.println(points_lon[i], 6);
  }
}

double get_rel_brng_update_dist(double lat2, double lon2)
{
  // http://www.movable-type.co.uk/scripts/latlong.html
  // Conversion factor from degrees to radians (pi/180)

  ss_gps.listen();
  while (ss_gps.available() > 0)
    gps.encode(ss_gps.read());

  if (gps.location.isValid()) // ********
  {
    lat1 = gps.location.lat();
    lon1 = gps.location.lng();

    // theta
    lat1 *= toRadian;
    lat2 *= toRadian;

    // labmda
    lon1 *= toRadian;
    lon2 *= toRadian;

    // Delta coordinates
    // double deltaLat_r = (lat2 - lat1);
    // double deltaLon_r = (lon2 - lon1);

    // Distance
    double a = sin((lat2 - lat1) / 2) * sin((lat2 - lat1) / 2) + cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2) * sin((lon2 - lon1) / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    double y = sin(lon2 - lon1) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
    double brng = atan2(y, x) * 57.3; // *******  +/-
    dist = 6371 * c * 1000;

    double heading = heading_compass(); // updates compass heading
    return (brng - heading);
  }
  else
  {
    return 1000;
  }
}

void motor(double relBearing)
{
  int left, right;
  if (relBearing > 0)
    left = LEFT_topSpeed;
  else
    left = (int)k_L * relBearing + LEFT_topSpeed;

  if (relBearing <= 0)
    right = RIGHT_topSpeed;
  else
    right = (int)-k_R * relBearing + RIGHT_topSpeed;

  // Serial.print("Left Motor: ");
  // Serial.print(left);
  // Serial.print("      Right Motor: ");
  // Serial.println(right);

  analogWrite(left_motor_pwm, left);
  analogWrite(right_motor_pwm, right);
}

void motor_bluetooth(int a, int b)
{
  bool l, r;
  if (a < 0)
    l = HIGH;
  else
    l = LOW;
  if (b < 0)
    r = HIGH;
  else
    r = LOW;
  //  a = constrain(a,-255,255);
  //  b = constrain(b,-255,255);
  if (abs(a) > 255)
    a = 255;
  if (abs(b) > 255)
    b = 255;
  digitalWrite(left_motor_A, l);
  digitalWrite(left_motor_B, !l);
  analogWrite(left_motor_pwm, abs(a));
  digitalWrite(right_motor_C, r);
  digitalWrite(right_motor_D, !r);
  analogWrite(right_motor_pwm, abs(b));
}

void autodrive()
{
  int index = 0;
  while (1)
  {
    double rel_brng = get_rel_brng_update_dist(points_lat[index], points_lon[index]);
    if (rel_brng == 1000)
    {
      motor_bluetooth(0, 0);
      ss_bluetooth.listen();
      ss_bluetooth.println("Waiting for GPS");
      delay(1000);
    }
    else
    {
      motor(rel_brng);
      if (dist < WAYPOINT_THRESHHOLD)
      {
        index++;
      }
      if (index >= no_points * 2)
      {
        motor_bluetooth(0, 0);
        break;
      }
      // ss_bluetooth.listen();
      // if (ss_bluetooth.available())
      // {
      //   char c = ss_bluetooth.read();
      //   if (c == 'm')
      //   {
      //     motor_bluetooth(0, 0);
      //   }
      // }
    }
    if (digitalRead(esp_pin) == 1)
    {
      digitalWrite(waste_pull_motor, HIGH);
    }
    else
    {
      digitalWrite(waste_pull_motor, LOW);
    }
  }
  auto_mode = false;
  ss_bluetooth.listen();
  ss_bluetooth.println("Select Mode (1 for autodrive, 2 for manual drive): ");
}

bool read_bluetooth_coordinates()
{
  int cnt = 0;
  String coord = "";

  while (1)
    if (ss_bluetooth.available())
    {
      char c = ss_bluetooth.read();
      if (c == '#')
      {
        if (coord.length() >= 1)
        {
          no_points = coord.toInt();
          coord = ""; // clears variable for new input
        }

        auto_mode = true;
        ss_bluetooth.print("Sweep value: ");
        ss_bluetooth.println(no_points);
        ss_bluetooth.println("Coordinates are:");
        for (int i = 0; i < 4; i++)
        {
          ss_bluetooth.print("coord-");
          ss_bluetooth.print(i + 1);
          ss_bluetooth.print(": Lat: ");
          ss_bluetooth.print(coordinates_lat[i], 5);
          ss_bluetooth.print(" Lng: ");
          ss_bluetooth.println(coordinates_lon[i], 5);
        }
        get_points();
        return false;
      }
      if (c == ',')
      {
        if (coord.length() > 1)
        {
          coordinates_lat[cnt] = coord.toDouble();
          coord = ""; // clears variable for new input
        }
      }
      else if (c == ';')
      {
        if (coord.length() > 1)
        {
          coordinates_lon[cnt] = coord.toDouble();
          cnt++;
          coord = ""; // clears variable for new input
        }
      }
      else
      {
        coord += c; // makes the string readString
      }
    }
  return true;
}

bool bluetooth_drive()
{
  if (ss_bluetooth.available())
  {
    char c = ss_bluetooth.read();
    if (c == 'w')
    {
      motor_bluetooth(LEFT_topSpeed, RIGHT_topSpeed);
    }
    else if (c == 's')
    {
      motor_bluetooth(-LEFT_topSpeed, -RIGHT_topSpeed);
    }
    else if (c == 'a')
    {
      motor_bluetooth(-LEFT_topSpeed, RIGHT_topSpeed);
    }
    else if (c == 'd')
    {
      motor_bluetooth(LEFT_topSpeed, -RIGHT_topSpeed);
    }
    else if (c == 'm')
    {
      motor_bluetooth(0, 0);
      ss_bluetooth.println("returning to mode");
      return false;
    }
    else
    {
      motor_bluetooth(0, 0);
      return true;
    }
  }
  motor_bluetooth(0, 0);
  return true;
}

void select_mode_bluetooth()
{
  ss_bluetooth.listen();
  if (ss_bluetooth.available())
  {
    char c = ss_bluetooth.read();
    if (c == '1')
    {
      ss_bluetooth.println("Enter coordinates");
      while (read_bluetooth_coordinates())
      {
      }
    }
    else if (c == '2')
    {
      ss_bluetooth.println("Drive mode: awsd for navigation, m for return to mode");
      while (bluetooth_drive())
      {
      }
    }
    else
    {
      ss_bluetooth.println("Invalid input! press (1/2)");
    }
  }
}

void setup()
{
  Serial.begin(9600);
  // i2c begin for HMC5883L compass sensor
  Wire.begin();
  compass_init();
  // Software serial for GPS
  ss_gps.begin(GPS_BAUD);
  ss_bluetooth.begin(bluetooth_BAUD);
  // led init
  pinMode(LED_pin, OUTPUT);

  // motor init
  pinMode(left_motor_pwm, OUTPUT);
  pinMode(right_motor_pwm, OUTPUT);
  pinMode(left_motor_A, OUTPUT);
  pinMode(left_motor_B, OUTPUT);
  pinMode(right_motor_C, OUTPUT);
  pinMode(right_motor_D, OUTPUT);
  pinMode(waste_pull_motor, OUTPUT);

  ss_bluetooth.listen();
  ss_bluetooth.println("Select Mode (1 for autodrive, 2 for manual drive): ");
}

void loop()
{
  // show_xy_compass();
  // show_head_compass();
  select_mode_bluetooth();
  if (auto_mode)
  {
    autodrive();
  }
  // delay(1000);
}