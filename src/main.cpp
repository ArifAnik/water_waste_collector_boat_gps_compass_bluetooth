#include <Arduino.h>

#include <EEPROMEx.h>

#include <Wire.h>

#include <TinyGPSplus.h>
#include <SoftwareSerial.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#define GPS_RX 12
#define GPS_TX 2
#define GPS_BAUD 9600
#define bluetooth_BAUD 9600
#define WAYPOINT_THRESHHOLD 4
#define LED_pin 13

#define dummy_lat 24.36728
#define dummy_lng 88.62554

#define left_f_pwm 3
#define left_b_pwm 5
#define left_ena 4
#define right_f_pwm 9
#define right_b_pwm 6
#define right_ena 7

#define waste_pull_motor 11
#define esp_pin 2

#define LEFT_topSpeed 200
#define RIGHT_topSpeed 200

unsigned long gpsReadStart = 0, gpsReadTime = 0, wasteReadTime = 0;

TinyGPSPlus gps;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

SoftwareSerial ss_gps(12, 10);
SoftwareSerial ss_bluetooth(A3, A2);

double k_L = LEFT_topSpeed / 180, k_R = RIGHT_topSpeed / 180;
double coordinates_lat[4], coordinates_lon[4];
int no_points = 10;
double points_lat[20];
double points_lon[20];
int sharp[10] = {1, 1, 2, 3, 4, 1, 2, 3, 4, 1};
double dist = 0;
const double toRadian = 0.01745329251;
const double toDegree = 57.2957795131;
bool auto_mode = false;
double llat, llon;

void compass_init()
{
  Wire.begin();
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
  // ss_bluetooth.listen();
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
  double lat1, lon1;
  // Serial.println("Reading GPS");
  ss_bluetooth.end();
  ss_gps.begin(GPS_BAUD);
  gpsReadTime = millis();
  while (millis() - gpsReadTime < 250)
  {
    if (ss_gps.available() > 0)
    {
      // Serial.println("Parsing GPS Data");
      gps.encode(ss_gps.read());
      // Serial.println(ss_gps.read());
    }
  }
  // Serial.println(gps.location.lat());
  if (gps.location.isValid()) // ********
  {
    // Serial.println("Co ordinate found");
    lat1 = gps.location.lat();
    llat = lat1;
    lon1 = gps.location.lng();
    llon = lon1;

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
    ss_gps.end();
    ss_bluetooth.begin(bluetooth_BAUD);
    // Serial.print("C: ");
    // Serial.println(heading);
    return (brng - heading);
  }
  else
  {
    ss_gps.end();
    ss_bluetooth.begin(bluetooth_BAUD);
    return 1000;
  }
}

void show_save_coord(int i)
{
  double lat1, lon1;
  bool k = false;
  // Serial.println("Reading GPS");
  ss_bluetooth.end();
  ss_gps.begin(GPS_BAUD);
  gpsReadTime = millis();
  while (millis() - gpsReadTime < 250)
  {
    if (ss_gps.available() > 0)
    {
      // Serial.println("Parsing GPS Data");
      gps.encode(ss_gps.read());
      // Serial.println(ss_gps.read());
    }
  }
  if (gps.location.isValid()) // ********
  {
    // Serial.println("Co ordinate found");
    lat1 = gps.location.lat();
    llat = lat1;
    lon1 = gps.location.lng();
    llon = lon1;
    // ss_gps.end();
    // ss_bluetooth.begin(bluetooth_BAUD);
    // Serial.print("C: ");
    // Serial.println(heading);
    k = true;
  }
  else
  {
    k = false;
  }
  ss_gps.end();
  ss_bluetooth.begin(bluetooth_BAUD);
  if (k)
  {
    ss_bluetooth.print("lat:");
    ss_bluetooth.print(llat, 6);
    ss_bluetooth.print("lon:");
    ss_bluetooth.println(llon, 6);
    coordinates_lat[i] = llat;
    coordinates_lon[i] = llon;
  }
}

void motor_bluetooth(int lf, int lb, int rf, int rb)
{
  analogWrite(left_f_pwm, lf);
  analogWrite(left_b_pwm, lb);
  analogWrite(right_f_pwm, rf);
  analogWrite(right_b_pwm, rb);
}

void motor(double relBearing)
{
  int left, right;
  if (relBearing == 0)
  {
    left = LEFT_topSpeed;
    right = RIGHT_topSpeed;
  }
  // else if(relBearing >60){
  //   left = LEFT_topSpeed;
  //   right = 0;
  // }
  // else if(relBearing <-60){
  //   left = 0;
  //   right = RIGHT_topSpeed;
  // }
  else
  {
    if (relBearing > 0)
      left = LEFT_topSpeed;
    else
      left = (int)k_L * relBearing + LEFT_topSpeed;

    if (relBearing < 0)
      right = RIGHT_topSpeed;
    else
      right = (int)-k_R * relBearing + RIGHT_topSpeed;
  }

  ss_bluetooth.print("Left:");
  ss_bluetooth.print(left);
  ss_bluetooth.print(" Right:");
  ss_bluetooth.println(right);

  if (left > 0 && right > 0)
    motor_bluetooth(left, 0, right, 0);
  else if (left > 0 && right < 0)
    motor_bluetooth(left, 0, 0, right);
  else if (left < 0 && right < 0)
    motor_bluetooth(0, left, right, 0);
  else if (left < 0 && right > 0)
    motor_bluetooth(0, left, right, 0);
}

void autodrive()
{
  digitalWrite(waste_pull_motor, HIGH);
  int index = 0;
  while (1)
  {
    double rel_brng = get_rel_brng_update_dist(coordinates_lat[index], coordinates_lon[index]);

    if (rel_brng == 1000)
    {
      // digitalWrite(waste_pull_motor, LOW);
      motor_bluetooth(0, 0, 0, 0);
      // ss_bluetooth.listen();
      ss_bluetooth.println("Waiting for GPS");
      delay(1000);
    }
    else
    {
      ss_bluetooth.print("R:");
      ss_bluetooth.print(rel_brng);
      ss_bluetooth.print(" D:");
      ss_bluetooth.print(dist);
      ss_bluetooth.print(" I:");
      ss_bluetooth.println(index);
      motor(rel_brng);
      if (index >= 4) // no_points * 2
      {
        ss_bluetooth.println("final found");
        // ss_bluetooth.println(index);
        motor_bluetooth(0, 0, 0, 0);
        break;
      }
      if (dist < WAYPOINT_THRESHHOLD)
      {
        ss_bluetooth.print("Coord found ");
        ss_bluetooth.println(index);
        motor_bluetooth(0, 0, 0, 0);
        delay(7000);
        // if (sharp[index] == 2)
        // {
        //   motor_bluetooth(255, 0);
        //   delay(900);
        //   motor_bluetooth(0, 0);
        // }
        // else if (sharp[index] == 4)
        // {
        //   motor_bluetooth(0, 255);
        //   delay(900);
        //   motor_bluetooth(0, 0);
        // }
        index++;
      }
      // if (rel_brng > 170 || rel_brng < -170)
      // {
      //   ss_bluetooth.println("coord found by rel brng");
      //   index++;
      // }

      // ss_bluetooth.listen();
      // if (ss_bluetooth.available())
      // {
      //   char c = ss_bluetooth.read();
      //   if (c == 'm')
      //   {
      //     ss_bluetooth.println("returning to main menu");
      //     motor_bluetooth(0, 0);
      //     break;
      //   }
      //   // else if (c == 'w')
      //   // {
      //   //   digitalWrite(waste_pull_motor, HIGH);
      //   // }
      //   // else if (c == 's')
      //   // {
      //   //   digitalWrite(waste_pull_motor, LOW);
      //   // }
      // }
    }
    if (digitalRead(esp_pin) == 1)
    {
      wasteReadTime = millis();
      digitalWrite(waste_pull_motor, LOW);
      // delay(5000);
    }
    if (millis() - wasteReadTime > 5000)
    {
      digitalWrite(waste_pull_motor, HIGH);
    }
  }

  auto_mode = false;
  // ss_bluetooth.listen();
  ss_bluetooth.println("Select Mode (1 for autodrive, 2 for manual drive): ");
}

void autodrive_coord()
{
  digitalWrite(waste_pull_motor, HIGH);
  int index = 0;
  while (1)
  {
    double rel_brng = get_rel_brng_update_dist(points_lat[index], points_lon[index]);

    if (rel_brng == 1000)
    {
      // digitalWrite(waste_pull_motor, LOW);
      motor_bluetooth(0, 0, 0, 0);
      // ss_bluetooth.listen();
      ss_bluetooth.println("Waiting for GPS");
      delay(1000);
    }
    else
    {
      ss_bluetooth.print("R:");
      ss_bluetooth.print(rel_brng);
      ss_bluetooth.print(" D:");
      ss_bluetooth.print(dist);
      ss_bluetooth.print(" I:");
      ss_bluetooth.println(index);
      motor(rel_brng);
      if (index >= 4) // no_points * 2
      {
        ss_bluetooth.println("final found");
        // ss_bluetooth.println(index);
        motor_bluetooth(0, 0, 0, 0);
        break;
      }
      if (dist < WAYPOINT_THRESHHOLD)
      {
        ss_bluetooth.print("Coord found ");
        ss_bluetooth.println(index);
        motor_bluetooth(0, 0, 0, 0);
        delay(7000);
        index++;
      }
    }
  }

  auto_mode = false;
  ss_bluetooth.println("Select Mode (1 for autodrive, 2 for manual drive): ");
}

bool read_bluetooth_coordinates()
{
  int cnt = 0;
  ss_gps.end();
  ss_bluetooth.begin(bluetooth_BAUD);
  while (1)
  {
    if (ss_bluetooth.available())
    {
      // while (ss_bluetooth.available())
      // {
      String c = ss_bluetooth.readString();
      Serial.println(c.length());
      Serial.print(c);
      String coord = "";
      for (int i = 0; i < (int)c.length(); i++)
      {
        if (c[i] == '#')
        {
          if (coord.length() >= 1)
          {
            // Serial.print("No point: ");
            // Serial.println(coord); // prints string to serial port out
            // Serial.print(';');   // prints delimiting ","
            no_points = coord.toInt();
            // Serial.println(no_points);
            // Serial.println(',');
            // cnt++;
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
            // Serial.print("coord-");
            // Serial.print(i + 1);
            // Serial.print(": Lat: ");
            // Serial.print(coordinates_lat[i], 5);
            // Serial.print(" Lng: ");
            // Serial.println(coordinates_lon[i], 5);
          }
          get_points();
          return false;
        }
        else if (c[i] == ',')
        {
          if (coord.length() > 1)
          {
            // Serial.print(coord); // prints string to serial port out
            // Serial.print("coorda"); // prints delimiting ","
            // Serial.
            // coordinates_lat[cnt] = coord.toDouble();
            coordinates_lat[cnt] = coord.toDouble();
            // Serial.print(coordinates_lat[cnt], 6);
            // Serial.println(',');

            coord = ""; // clears variable for new input
          }
        }
        else if (c[i] == ';')
        {
          if (coord.length() > 1)
          {
            // Serial.print("coordg"); // prints string to serial port out
            // Serial.print(';');   // prints delimiting ","
            coordinates_lon[cnt] = coord.toDouble();
            // Serial.print(coordinates_lon[cnt], 6);
            // Serial.println(',');
            cnt++;
            coord = ""; // clears variable for new input
          }
        }
        else
        {
          coord += c[i]; // makes the string readString
        }
      }
    }
  }
  // Serial.println("nn");
  return true;
}

bool bluetooth_drive()
{
  if (ss_bluetooth.available())
  {
    char c = ss_bluetooth.read();
    if (c == 'w')
    {
      motor_bluetooth(LEFT_topSpeed, 0, RIGHT_topSpeed, 0);
    }
    else if (c == 's')
    {
      motor_bluetooth(0, LEFT_topSpeed, 0, RIGHT_topSpeed);
    }
    else if (c == 'a')
    {
      motor_bluetooth(0, LEFT_topSpeed, RIGHT_topSpeed, 0);
    }
    else if (c == 'd')
    {
      motor_bluetooth(LEFT_topSpeed, 0, 0, RIGHT_topSpeed);
    }
    else if (c == 'm')
    {
      motor_bluetooth(0, 0, 0, 0);
      ss_bluetooth.println("returning to mode");
      return false;
    }
    else if (c == 'e')
    {
      digitalWrite(waste_pull_motor, HIGH);
    }
    else if (c == 'r')
    {
      digitalWrite(waste_pull_motor, LOW);
    }
    else if (c == 'z')
    {
      ss_bluetooth.println("saving coord 0");
      show_save_coord(0);
    }
    else if (c == 'x')
    {
      ss_bluetooth.println("saving coord 1");
      show_save_coord(1);
    }
    else if (c == 'c')
    {
      ss_bluetooth.println("saving coord 2");
      show_save_coord(2);
    }
    else if (c == 'v')
    {
      ss_bluetooth.println("saving coord 3");
      show_save_coord(3);
    }
    else if (c == 'b')
    {
      ss_bluetooth.println("Showing coords");
      for (int i = 0; i < 4; i++)
      {
        ss_bluetooth.print(i);
        ss_bluetooth.print(": lat-");
        ss_bluetooth.print(coordinates_lat[i]);
        ss_bluetooth.print(" lon-");
        ss_bluetooth.println(coordinates_lon[i]);
      }
    }
    else
    {
      digitalWrite(waste_pull_motor, HIGH);
      motor_bluetooth(0, 0, 0, 0);
      return true;
    }
  }
  // motor_bluetooth(0, 0);
  return true;
}

void select_mode_bluetooth()
{
  // ss_bluetooth.listen();

  if (ss_bluetooth.available())
  {
    char c = ss_bluetooth.read();
    if (c == '1')
    {
      Serial.println("1");
      ss_bluetooth.println("Enter coordinates");
      while (read_bluetooth_coordinates())
      {
      }
    }
    else if (c == '2')
    {
      Serial.println("2");
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

void show_motor(double relBearing)
{
  int left, right;
  if (relBearing == 0)
  {
    left = LEFT_topSpeed;
    right = RIGHT_topSpeed;
  }
  else
  {
    if (relBearing > 0)
      left = LEFT_topSpeed;
    else
      left = (int)k_L * relBearing + LEFT_topSpeed;

    if (relBearing < 0)
      right = RIGHT_topSpeed;
    else
      right = (int)-k_R * relBearing + RIGHT_topSpeed;
  }

  ss_bluetooth.print("Left: ");
  ss_bluetooth.print(left);
  ss_bluetooth.print("  Right: ");
  ss_bluetooth.println(right);
}

int ind = 0;
void show_rel_dist()
{
  double rel = get_rel_brng_update_dist(coordinates_lat[ind], coordinates_lon[ind]);
  if (rel == 1000)
  {
    ss_bluetooth.println("wait for gps");
    double h = heading_compass();
    ss_bluetooth.print("C");
    ss_bluetooth.println(h);
  }
  else
  {
    if (dist < WAYPOINT_THRESHHOLD)
    {
      ss_bluetooth.print("point found: ");
      ss_bluetooth.println(ind);
      ind++;
    }
    if (ind >= no_points)
    {
      ind = 0;
      ss_bluetooth.println("final point reached");
      auto_mode = false;
    }
    ss_bluetooth.print("R: ");
    ss_bluetooth.print(rel);
    ss_bluetooth.print("  D: ");
    ss_bluetooth.println(dist);
    ss_bluetooth.print("lat:");
    ss_bluetooth.print(llat);
    ss_bluetooth.print(" lon:");
    ss_bluetooth.println(llon);
    show_motor(rel);
    // motor(rel);
  }
  delay(1500);
}

void setup()
{
  Serial.begin(9600);
  Serial.println("I am on");
  // i2c begin for HMC5883L compass sensor
  compass_init();
  // Software serial for GPS
  // ss_gps.begin(GPS_BAUD);
  ss_bluetooth.begin(bluetooth_BAUD);
  // led init
  pinMode(LED_pin, OUTPUT);

  // motor init
  for (int i = 3; i < 10; i++)
  {
    pinMode(i, OUTPUT);
  }

  digitalWrite(left_ena, HIGH);
  digitalWrite(right_ena, HIGH);
  analogWrite(left_f_pwm, 0);
  analogWrite(left_b_pwm, 0);
  analogWrite(right_f_pwm, 0);
  analogWrite(right_b_pwm, 0);

  pinMode(waste_pull_motor, OUTPUT);
  digitalWrite(waste_pull_motor, HIGH);

  pinMode(esp_pin, INPUT);

  // ss_bluetooth.listen();
  ss_bluetooth.println("Select Mode (1 for autodrive, 2 for manual drive): ");
}

void loop()
{
  select_mode_bluetooth();
  if (auto_mode)
  {
    autodrive();
  }
  // Serial.println(heading_compass());
  // delay(1000);
  // if (auto_mode)
  // {
  //   show_rel_dist();
  // }
  // delay(1000);
}