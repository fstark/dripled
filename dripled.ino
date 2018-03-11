#include <I2Cdev.h>
#include <MPU6050.h>
#include <LedControl.h>

MPU6050 accelgyro;
LedControl lc=LedControl(12,10,11,1);

#define FIXED

#define SIZEX 8
#define SIZEY 8

template <typename T> class vec
{
  T x_;
  T y_;

public:
  vec() : x_{0}, y_{0} {}
  vec( int16_t x, int16_t y ) : x_{(T)x}, y_{(T)y} {}
  vec( T x, T y ) : x_{x}, y_{y} {}

  T x() const { return x_; }
  T y() const { return y_; }

  vec operator+( const vec&o ) const
  {
    return vec{ x()+ o.x(), y()+o.y() };
  }

  vec operator+=( const vec &o )
  {
    x_ += o.x();
    y_ += o.y();
    return *this;
  }

  vec operator*( T v ) const
  {
    return vec{ x()*v, y()*v };
  }

  vec operator*=( T v )
  {
    x_ *= v;
    y_ *= v;
    return *this;
  }

  vec operator/( T v ) const
  {
    if (x_==T{0} && y_==T{0})
      return *this;
    return vec{ x()/v, y()/v };
  }

  vec operator/=( T v )
  {
    if (x_==0 && y_==0)
      return *this;
    x_ /= v;
    y_ /= v;
    return *this;
  }

  T norm() const
  {
    return T{ sqrt( x()*x()+y()*y() ) };
  }
};

#ifdef FIXED

#define SCALEf 256.0

class fixed
{
  union
  {
    int16_t v_;
    struct
    {
      uint8_t l;
      int8_t h;
    };
  };

  void check() const
  {
    if (fabs(v_)>65536) Serial.println( "OVERFLOW" );
  }
public:
  fixed() : v_{0} {}
  
  explicit fixed( float f ) : v_{ f*SCALEf } {}

  bool operator==( const fixed &o ) const { return v_==o.v_; }

  fixed operator+( const fixed &o ) const { fixed r; r.v_ = v_+o.v_; check(); return r; }

  fixed operator+=( const fixed &o ) { v_ += o.v_; check(); return *this; }

//  fixed operator+=( double v ) { v_ += v*SCALEf; check(); return *this; }
  

  fixed operator-( const fixed &o ) const { fixed r; r.v_ = v_-o.v_; check(); return r; }
  
  fixed operator-( void ) const { fixed r; r.v_ = -v_; check(); return r; }

  fixed operator-=( const fixed &o ) { v_ -= o.v_; check(); return *this; }

//  fixed operator-=( double v ) { v_ -= v*SCALEf; check(); return *this; }
  
  fixed operator*( double v ) const { fixed r; r.v_ = v_*v; check(); return r; }
  
//  fixed operator*( fixed o ) { fixed r; r.v_ = (v_/SCALEf)*o.v_; check(); return r; }
  fixed operator*( const fixed &o ) const
  {
//    int8_t h0 = h;
//    uint8_t l0 = l;
//
//    int8_t h1 = o.h;
//    uint8_t l1 = o.l;
//
//    uint16_t v = 0;
//
//    if (l0 && l1)
//      v += (l0*l1)>>8;
//    if (l0 && h1)
//      v += l0*h1;
//    if (h0 && l1)
//      v += h0*l1;
//    if (h0 && h1)
//      v += (h0*h1)<<8;
//    
    long v = v_;
    v *= o.v_;
    v >>= 8;
    fixed r;
    r.v_ = v;
    check();
    return r;
  }

  fixed operator*=( double v ) { v_ *= v; check(); return *this; }
 
  fixed operator/( double v ) const { fixed r; r.v_ = v_/v; check(); return r; }
  
  fixed operator/( const fixed &o ) const { fixed r; r.v_ = v_*SCALEf/o.v_; check(); return r; }
  
  fixed operator/=( double v ) { v_ /= v; check(); return *this; }
  
//  bool operator>( int v ) { return v_ > v*SCALEf; }

  bool operator>( const fixed &o ) const { return v_ > o.v_; }

  operator float() const { return v_/SCALEf; }

};

#else

typedef float fixed;

#endif

typedef vec<fixed> fixed_vec;


bool insidex( int i ) { return (i>=0 && i<SIZEX); }
bool insidey( int i ) { return (i>=0 && i<SIZEY); }

const fixed MAX_MASS{1};


class field
{
  struct
  {
    fixed mass_;
    fixed_vec speed_; 
    fixed diff[4];    //  Diffusion
  } elements_[SIZEX][SIZEY];

public:
  field()
  {
    for (int x=0;x!=SIZEX;x++)
      for (int y=0;y!=SIZEY;y++)
      {
        elements_[x][y] = { fixed{0.5} ,{0,0}};
      }
//    elements_[0][1] = {18,{0,0}};
//    elements_[3][1] = {5,{0,0}};
  }

  void display()
  {
    for (int x=0;x!=SIZEX;x++)
      for (int y=0;y!=SIZEY;y++)
        lc.setLed(0,SIZEX-(x+.5),SIZEY-(y+.5),elements_[x][y].mass_>=0.5);
  }

  void step( const fixed_vec &acc )
  {
    int static dx[4] = { -1, 0, 1, 0 };
    int static dy[4] = {  0, 1, 0,-1 };

    //  Compute how mass will move out of each cell
    for (int x=0;x!=SIZEX;x++)
      for (int y=0;y!=SIZEY;y++)
      {
        auto &e = elements_[x][y];
        e.speed_ += acc;

        if (e.speed_.norm()>1)
           e.speed_/=e.speed_.norm();

        fixed m = e.mass_;
        fixed_vec s = e.speed_;

          //  Basic diffusion
        fixed a = s.norm()/(fabs(s.x())+fabs(s.y()));
        for (int d = 0;d!=4;d++)
        {
            e.diff[d] = m*0.01;
//            e.diff[d] = 0;
        }

       auto tmp = a*m*0.99;

          //  Movment of mass due to speed
        if (s.x()<0) e.diff[0] -= s.x()*tmp;
        if (s.y()>0) e.diff[1] += s.y()*tmp;
        if (s.x()>0) e.diff[2] += s.x()*tmp;
        if (s.y()<0) e.diff[3] -= s.y()*tmp;

          //  Don't move mass outside of the field
        for (int d = 0;d!=4;d++)
        {
          int nx = x + dx[d];
          int ny = y + dy[d];
          if (!insidex(nx) || !insidey(ny))
             e.diff[d] = fixed{0};
        }
      }

      //  Move matter between cells
    for (int x=0;x!=SIZEX;x++)
      for (int y=0;y!=SIZEY;y++)
      {
        fixed_vec toto;
        
        for (int d = 0;d!=4;d++)
        {
          int nx = x + dx[d];
          int ny = y + dy[d];

          auto &e = elements_[x][y];

          //  Update mass
          if (e.diff[d]>0)
          {
            auto &e1 = elements_[nx][ny];
            e1.mass_ += e.diff[d];
            if (e1.mass_>MAX_MASS)
            {
              e.diff[d] -= e1.mass_- MAX_MASS;
              e1.mass_ = MAX_MASS;
            }
            e.mass_ -= e.diff[d];
            if (e.mass_<0)
              e.mass_ = fixed{0};  //  ####

            if (e.mass_<0)
            {
              Serial.print( "ERROR MASS : " );
              Serial.print( e.mass_ );
              Serial.print( "<0 SPEED : {" );
              Serial.print( e.speed_.x() );
              Serial.print( "," );
              Serial.print( e.speed_.y() );
              Serial.print( "} DIFFS : " );
              for (int dd=0;dd!=4;dd++)
              {
                Serial.print( e.diff[dd] );
                Serial.print( "/" );
              }
              
              Serial.println();
            }
          }
          toto += fixed_vec{ dx[d], dy[d] }*e.diff[d];
        }
      }

    for (int x=0;x!=SIZEX;x++)
      for (int y=0;y!=SIZEY;y++)
      {
        auto &e = elements_[x][y];

        fixed_vec out;
        fixed delta_out_m{ 0 };
        
        for (int d = 0;d!=4;d++)
        {
          out += fixed_vec{ dx[d], dy[d] }*e.diff[d];
          delta_out_m  += e.diff[d];
        }

        out /= delta_out_m;
        
        fixed_vec in;
        fixed delta_in_m{ 0 };

        for (int d = 0;d!=4;d++)
        {
          int nx = x+dx[d];
          int ny = y+dy[d];
          if (insidex(nx)&&insidey(ny))
          {
            auto &e1 = elements_[nx][ny];

            in += fixed_vec{ -dx[d], -dy[d] }*e1.diff[d];
            delta_in_m  += e1.diff[d];
          }
        }

        if (e.mass_!=0)
          e.speed_ = (in+out*(e.mass_-delta_in_m))/e.mass_;
        else
          e.speed_ = {0,0};
      }
  }

  void debug()
  {
    for (int x=0;x!=SIZEX;x++)
    {
      for (int y=0;y!=SIZEY;y++)
      {
        auto &e = elements_[x][y];
        Serial.print( e.mass_ );
        Serial.print( "[" );
        Serial.print( e.speed_.x() );
        Serial.print( "," );
        Serial.print( e.speed_.y() );
        Serial.print( "]\t" );
      }
        Serial.println();
    }
    Serial.println();
  }
};

field world;

void setup()
{
  Serial.begin(115200);
 
  Serial.println("Init LED array");

  lc.shutdown(0,false);
  lc.setIntensity(1,0);
  lc.clearDisplay(0);
  
  Serial.println("Init I2C");

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.println("Init MPU6050");
    accelgyro.initialize();

    accelgyro.setXAccelOffset(-1250);
    accelgyro.setYAccelOffset(-759);
    accelgyro.setZAccelOffset(1241);
//    accelgyro.setXGyroOffset(-812);
//    accelgyro.setYGyroOffset(30);
//    accelgyro.setZGyroOffset(-22);
}

void loop()
{
  int16_t ax, ay, az;
  accelgyro.getAcceleration(&ax, &ay, &az);
  fixed_vec acc{ ay/8000, -az/8000 };

//  world.debug();
  auto start = millis();
  world.step( acc );
  auto duration = millis()-start;
//  Serial.print( duration );
//  Serial.print( " " );
  world.display();
//  duration = millis()-start;
  Serial.println( duration );
//  delay( 1000 );
 
//  Serial.print("a/g:\t");
//  Serial.print(ax); Serial.print(" \t");
//  Serial.print(ay); Serial.print(" \t");
//  Serial.print(az); Serial.println(" \t");
}

