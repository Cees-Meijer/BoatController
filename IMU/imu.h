#pragma once

#include <cstdint>
#include <tuple>


typedef struct _Vec
{
	float X;
	float Y;
	float Z;
}Vec;

class imu
{
public:
  virtual void read_raw()
  {
    read_mag_raw();
    read_acc_raw();
    read_gyro_raw();
  }

  virtual void read_acc_raw() = 0;
  virtual void read_mag_raw() = 0;
  virtual void read_gyro_raw() = 0;

  virtual float get_acc_scale() const = 0;
  virtual float get_gyro_scale() const = 0;

  int32_t m[3];
  int32_t a[3];
  int32_t g[3];

  // TODO: remove stuff below this point

  // Scaled readings
  virtual std::tuple<float,float,float>read_mag() = 0;  // In body coords, scaled to -1..1 range
  virtual std::tuple<float,float,float>read_acc() = 0;  // In body coords, with units = g
 // virtual void read_acc(Vec*);  // In body coords, with units = g

  virtual std::tuple<float,float,float>read_gyro() = 0; // In body coords, with units = rad/sec
  void read(){ read_mag(); read_acc(); read_gyro(); }

  virtual void measure_offsets() = 0;
  virtual void enable() = 0;
  virtual void load_calibration() = 0;

  long gyro_offset[3];
  std::tuple<int,int,int> mag_min, mag_max;


};
