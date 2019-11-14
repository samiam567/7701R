#ifndef GRAPHICS_H
#define GRAPHICS_H

std::vector<double> calculateRotation(double xComponent, double yComponent, double angle);

namespace HEEA_Graphics {

  class Physics_object {
    protected:
      double x,y,z,q;

    public:
    Physics_object(double x1, double y1, double z1, double q1) : x{x1}, y{y1}, z{z1}, q{q1} {}

    void setPos(double x1, double y1, double z1, double q1) {
      x = x1;
      y = y1;
      z = z1;
      q = q1;
    }

    double getX() {
      return x;
    }

    double getY() {
      return y;
    }

    double getZ() {
      return z;
    }

    double getQ() {
      return q;
    }


  };

  class Point : public Physics_object {
      public:
        double xi,yi,zi,qi; //these must be set by the shape

        Point(double x1, double y1, double z1, double q1) : Physics_object(x1,y1,z1,q1) {}

        void setIs(double xi1, double yi1, double zi1, double qi1) {
          xi = xi1;
          yi = yi1;
          zi = zi1;
          qi = qi1;
        }

        double getXI() {
          return xi;
        }

        double getYI() {
          return yi;
        }

        double getZI() {
          return zi;
        }

        double getQI() {
          return qi;
        }

  };

  class Shape : public Physics_object {
    protected:
      double xSpeed=0,ySpeed=0,zSpeed=0,qSpeed=0;
      double xAccel=0,yAccel=0,zAccel=0,qAccel=0;
      double xSize=0, ySize=0, zSize=0, qSize=0;
      bool customPointRenderOrder;
      std::vector<Point> points;
      std::vector<int> pointRenderOrder;
      std::string name = "unnamed";
      lv_obj_t*renderLine;

      void checkForBorderBounce() { //check if the object is leaving the screen and if so bounce it back
        if (x < xSize) {
          xSpeed = fabs(xSpeed);
        }else if (x > (480 - xSize)) {
          xSpeed = -fabs(xSpeed);
        }
        if (y < ySize) {
          ySpeed = fabs(ySpeed);
        }else if (y > (240 - ySize)) {
          ySpeed = -fabs(ySpeed);
        }
      }

      void calculateSize() {
        xSize = 0;
        ySize = 0;
        zSize = 0;
        qSize = 0;

        for (int i = 0; i < points.size(); i++) {
          if (fabs(points[i].getX()) > xSize) xSize = fabs(points[i].getX());
          if (fabs(points[i].getY()) > ySize) ySize = fabs(points[i].getY());
          if (fabs(points[i].getZ()) > zSize) zSize = fabs(points[i].getZ());
          if (fabs(points[i].getQ()) > qSize) qSize = fabs(points[i].getQ());
        }
        xSize *= 2;
        ySize *= 2;
        zSize *= 2;
        qSize *= 2;

        lv_obj_set_size(renderLine,xSize, ySize);
      }

    public:
      Shape(lv_obj_t * renderLine1, std::vector<Point> points1) : Physics_object(0,0,0,0), renderLine(renderLine1), points(points1) {
        customPointRenderOrder = false;
        calculateSize();
        updateLine();
      }

      Shape(lv_obj_t * renderLine1, std::vector<Point> points1, std::vector<int> pointRenderOrder1) : Physics_object(0,0,0,0), points(points1), renderLine(renderLine1), pointRenderOrder(pointRenderOrder1)  {
        customPointRenderOrder = true;
        calculateSize();
        updateLine();
      }

      void setSpeed(double xSpeed1, double ySpeed1, double zSpeed1, double qSpeed1)  {
        xSpeed = xSpeed1;
        ySpeed = ySpeed1;
        zSpeed = zSpeed1;
        qSpeed = qSpeed1;
      }


      void updateLine() { //update the LVGL line with the object's 2d point coordinates

        static lv_point_t line_points[80];

        if (customPointRenderOrder) {
          for (int i = 0; i < pointRenderOrder.size(); i++) {
            line_points[i].x = static_cast<lv_coord_t>(x + points[pointRenderOrder[i]].getX());
            line_points[i].y = static_cast<lv_coord_t>(y + points[pointRenderOrder[i]].getY());
          }
          lv_line_set_points(renderLine, line_points, pointRenderOrder.size()); //Set the points
        }else{
          int i = 0;
          for (Point p : points) {
              line_points[i].x = static_cast<lv_coord_t>(x + p.getX());
              line_points[i].y = static_cast<lv_coord_t>(y + p.getY());
              i++;
          }
          lv_line_set_points(renderLine, line_points, points.size()); //Set the points
        }
      }

      void update(double frames) {
        checkForBorderBounce();
        updateLine();
        x += xSpeed * frames;
        y += ySpeed * frames;
        z += zSpeed * frames;
        q += qSpeed * frames;

        xSpeed += xAccel * frames;
        ySpeed += yAccel * frames;
        zSpeed += zAccel * frames;

      }




  };

  class RotatableShape : public Shape {
    protected:
      double xRotation=0,yRotation=0,zRotation=0,qXRotation=0, qYRotation=0, qZRotation=0;
      double prevXRotation=0,prevYRotation=0,prevZRotation=0,prevQXRotation=0, prevQYRotation=0, prevQZRotation=0;
      double angularVelocityX=0, angularVelocityY=0, angularVelocityZ=0,angularVelocityQX=0,angularVelocityQY=0,angularVelocityQZ=0;
      double angularAccelX=0, angularAccelY=0, angularAccelZ=0,angularAccelQX=0,angularAccelQY=0,angularAccelQZ=0;

    protected:
      void rotatePoints() {
          std::vector<double> rotComponents;
          for (int i = 0; i < points.size(); i++) {
  					//zRotation
  					rotComponents = calculateRotation(points[i].getX(),points[i].getY(),zRotation - prevZRotation);
  					points[i].setPos( rotComponents[0],rotComponents[1],points[i].getZ(),points[i].getQ());

  					//xRotation
  					rotComponents = calculateRotation(points[i].getZ(),points[i].getY(),xRotation - prevXRotation);
  					points[i].setPos(points[i].getX(),  rotComponents[1], rotComponents[0],points[i].getQ());

  					//yRotation
  					rotComponents = calculateRotation(points[i].getX(),points[i].getZ(),yRotation - prevYRotation);
  					points[i].setPos(rotComponents[0], points[i].getY()  ,rotComponents[1],points[i].getQ());

            //qXRotation
  					rotComponents = calculateRotation(points[i].getX(),points[i].getQ(),qXRotation - prevQXRotation);
  					points[i].setPos(rotComponents[0], points[i].getY() ,points[i].getZ(),rotComponents[1]);

            //qYRotation
  					rotComponents = calculateRotation(points[i].getY(),points[i].getQ(),qYRotation - prevQYRotation);
  					points[i].setPos(points[i].getX(),rotComponents[0] ,points[i].getZ(),rotComponents[1]);

            //qZRotation
  					rotComponents = calculateRotation(points[i].getZ(),points[i].getQ(),qZRotation - prevQZRotation);
  					points[i].setPos(points[i].getX(),points[i].getY(), rotComponents[0], rotComponents[1]);
          }

          //set prevRotations to this rotation as it the the last calculated rotation
          prevXRotation = xRotation;
          prevYRotation = yRotation;
          prevZRotation = zRotation;
          prevQXRotation = qXRotation;
          prevQYRotation = qYRotation;
          prevQZRotation = qZRotation;
      }


    public:
      //constructors
      RotatableShape(lv_obj_t*renderLine1, std::vector<Point> points1) : Shape(renderLine1,points1) {}
      RotatableShape(lv_obj_t*renderLine1, std::vector<Point> points1, std::vector<int> pointRenderOrder1) : Shape(renderLine1,points1,pointRenderOrder1) {}

    void update(double frames) {
      Shape::update(frames);
      rotatePoints();
      xRotation += angularVelocityX * frames;
      yRotation += angularVelocityY * frames;
      zRotation += angularVelocityZ * frames;

      qXRotation += angularVelocityQX * frames;
      qYRotation += angularVelocityQY * frames;
      qZRotation += angularVelocityQZ * frames;

      angularVelocityX += angularAccelX * frames;
      angularVelocityY += angularAccelY * frames;
      angularVelocityZ += angularAccelZ * frames;

      angularVelocityQX += angularAccelQX * frames;
      angularVelocityQY += angularAccelQY * frames;
      angularVelocityQZ += angularAccelQZ * frames;

    }

    void setRotation(double rotX, double rotY, double rotZ, double rotQX, double rotQY, double rotQZ) {
      xRotation = rotX;
      yRotation = rotY;
      zRotation = rotZ;

      qXRotation = rotQX;
      qYRotation = rotQY;
      qZRotation = rotQZ;
    }

    void setAngularVelocity(double angVX, double angVY, double angVZ, double angVQX, double angVQY, double angVQZ) {
  		angularVelocityX = angVX;
  		angularVelocityY = angVY;
  		angularVelocityZ = angVZ;

		  angularVelocityQX = angVQX;
      angularVelocityQY = angVQY;
      angularVelocityQZ = angVQZ;

    }

    void setAngularAccel(double angAccelX, double angAccelY, double angAccelZ, double angAccelQX, double angAccelQY, double angAccelQZ) {
  		angularAccelX = angAccelX;
  		angularAccelY = angAccelY;
  		angularAccelZ = angAccelZ;
      angularAccelQX = angAccelQX;
      angularAccelQY = angAccelQY;
      angularAccelQZ = angAccelQZ;

	   }

  };

}

void initializeLVGL();
void sayHello();
void updateStyle();
void makeConsole();
HEEA_Graphics::RotatableShape makeShape();

#endif
