#include <I2Cdev.h>
#include <MPU6050.h>
#include <LedControl.h>

MPU6050 accelgyro;
LedControl lc=LedControl(12,10,11,1);

#define SIZEX 6
#define SIZEY 6

class vec
{
  float x_;
  float y_;

public:
  vec() : x_{0}, y_{0} {}
  vec( int16_t x, int16_t y ) : x_{(float)x}, y_{(float)y} {}
  vec( float x, float y ) : x_{x}, y_{y} {}

  float x() const { return x_; }
  float y() const { return y_; }

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

  vec operator*( float v ) const
  {
    return vec{ x()*v, y()*v };
  }

  vec operator*=( float v )
  {
    x_ *= v;
    y_ *= v;
    return *this;
  }

  vec operator/( float v ) const
  {
    if (x_==0 && y_==0)
      return *this;
    return vec{ x()/v, y()/v };
  }

  vec operator/=( float v )
  {
    if (x_==0 && y_==0)
      return *this;
    x_ /= v;
    y_ /= v;
    return *this;
  }

  float norm() const
  {
    return sqrt( x()*x()+y()*y() );
  }
};

bool insidex( int i ) { return (i>=0 && i<SIZEX); }
bool insidey( int i ) { return (i>=0 && i<SIZEY); }

void plot( const vec& p, bool on_off )
{
  lc.setLed(0,1+SIZEX-p.x(),1+SIZEY-p.y(),on_off);
}

#define MAX_MASS  1

class field
{
  struct
  {
    float mass_;
    vec speed_; 
    float diff[4];    //  Diffusion
  } elements_[SIZEX][SIZEY];

public:
  field()
  {
    for (int x=0;x!=SIZEX;x++)
      for (int y=0;y!=SIZEY;y++)
      {
        elements_[x][y] = {0.5,{0,0}};
      }
//    elements_[0][1] = {18,{0,0}};
//    elements_[3][1] = {5,{0,0}};
  }

  void display()
  {
    for (int x=0;x!=SIZEX;x++)
      for (int y=0;y!=SIZEY;y++)
          plot( {x+.5f,y+.5f}, elements_[x][y].mass_>=0.5 );
  }

  void step( const vec &acc )
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

        auto m = e.mass_;
        auto s = e.speed_;

          //  Basic diffusion
        auto a = s.norm()/(fabs(s.x())+fabs(s.y()));
        for (int d = 0;d!=4;d++)
        {
            e.diff[d] = 0.01*m;
//            e.diff[d] = 0;
        }

          //  Movment of mass due to speed
        if (s.x()<0) e.diff[0] += -s.x()*0.99*a*m;
        if (s.y()>0) e.diff[1] += s.y()*0.99*a*m;
        if (s.x()>0) e.diff[2] += s.x()*0.99*a*m;
        if (s.y()<0) e.diff[3] += -s.y()*0.99*a*m;

          //  Don't move mass outside of the field
        for (int d = 0;d!=4;d++)
        {
          int nx = x + dx[d];
          int ny = y + dy[d];
          if (!insidex(nx) || !insidey(ny))
             e.diff[d] = 0;
        }
      }

      //  Move matter between cells
    for (int x=0;x!=SIZEX;x++)
      for (int y=0;y!=SIZEY;y++)
      {
        vec toto;
        
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
              e.diff[d] -= e1.mass_-MAX_MASS;
              e1.mass_ = MAX_MASS;
            }
            e.mass_ -= e.diff[d];
            if (e.mass_<0)
            {
              Serial.print( "ERROR : " );
              Serial.print( e.speed_.x() );
              Serial.print( "," );
              Serial.print( e.speed_.y() );
              Serial.print( " " );
              for (int dd=0;dd!=4;dd++)
              {
                Serial.print( e.diff[dd] );
                Serial.print( "/" );
              }
              
              Serial.println();
            }
          }
          toto += vec{ dx[d], dy[d] }*e.diff[d];
        }
      }

    for (int x=0;x!=SIZEX;x++)
      for (int y=0;y!=SIZEY;y++)
      {
        auto &e = elements_[x][y];

        vec out;
        float delta_out_m = 0;
        
        for (int d = 0;d!=4;d++)
        {
          out += vec{ dx[d], dy[d] }*e.diff[d];
          delta_out_m  += e.diff[d];
        }

        out /= delta_out_m;
        
        vec in;
        float delta_in_m = 0;

        for (int d = 0;d!=4;d++)
        {
          int nx = x+dx[d];
          int ny = y+dy[d];
          if (insidex(nx)&&insidey(ny))
          {
            auto &e1 = elements_[nx][ny];

            in += vec{ -dx[d], -dy[d] }*e1.diff[d];
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
  vec acc{ ay, -az };
  acc = acc/8000.0;

  world.step( acc );
  world.display();

 
//  Serial.print("a/g:\t");
//  Serial.print(ax); Serial.print(" \t");
//  Serial.print(ay); Serial.print(" \t");
//  Serial.print(az); Serial.println(" \t");
}


