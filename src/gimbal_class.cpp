#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>

//cooridinate frame class
class Frame{
  public:
    //rpy and xyz corresponds to URDF origin
    Eigen::Vector3d rpy;
    Eigen::Vector3d xyz;
    Eigen::Vector3d axis;
    double angle=0;
    bool isRev = false;
    Frame *parent = NULL; //pointer to parent frame

    Frame(){
      rpy.setZero();
      xyz.setZero();
    }

    Eigen::Vector3d getPos(){
      return xyz;
    }

    //this method returns the rotation matrix of the Frame/link
    Eigen::Matrix3d getRot(){
      double roll = origin.rpy[0], pitch = origin.rpy[1], yaw = origin.rpy[2];
      Eigen::Matrix3d Rx, Ry, Rz, R;
      Rx << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);
      Ry << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);
      Rz << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
      R = Rz*Ry*Rx;

      if(isRev){
        R = R * Eigen::AngleAxisd(angle, axis).toRotationMatrix();  //add joint rotation for revolute joint frames
      }
      return R;
    }
};

class Gimbal{
  public:
    void setstate(double yaw, double roll, double pitch){ //in order of motors
      yaw.angle = yaw;
      roll.angle = roll;
      pitch.angle = pitch;
    }
    //getDHmatrix() method returns the DH matrix from camera to base frame
    Eigen::MatrixXd getDHmatrix_L(){
      DH.block(0,0,3,3) = getRot(&Lcam);
      DH.block(0,3,3,1) = getPos(&Lcam);
      return DH;
    }
    Eigen::MatrixXd getDHmatrix_R(){
      DH.block(0,0,3,3) = getRot(&Rcam);
      DH.block(0,3,3,1) = getPos(&Rcam);
      return DH;
    }

  Gimbal(){
    //hard code gimbal configuration (URDF Format)
    base.xyz << 0,0,0;
    base.rpy << 0,0,0;

    yaw.isRev = true;
    yaw.xyz << 0.1, 0, 0,1;
    yaw.rpy << 0, pi/2, 0;
    yaw.axis << 0,0,1;
    yaw.parent = &base;

    roll.isRev = true;
    roll.xyz << -0.1, 0, 0.1;
    roll.rpy << 0, -pi/2, 0;
    roll.axis << 1,0,0;
    roll.parent = &yaw;

    pitch.isRev = true;
    pitch.xyz << 0.1, 0, 0;
    pitch.rpy << 0,0,0;
    pitch.axis << 0,1,0;
    pitch.parent = &roll;

    Lcam.xyz << ;
    Lcam.rpy << ;
    Lcam.parent = &pitch;

    Rcam.xyz << ;
    Rcam.rpy << ; 
    Rcam.parent = &pitch;
  }

  private:
    Eigen::Vector3d getPos(Frame* frame){
      Eigen::Vector3d pos = frame->getPos();
      frame = frame->parent;
      while(frame != NULL){
        pos = frame->getRot() * pos + frame->getPos();
      }
      return pos;
    }

    Eigen::Vector3d getRot(Frame* frame){
      Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
      while(frame != NULL){
        R = frame->getRot() * R;
        frame = frame->parent;
      }
      return pos;
    }
  
  private:
    Frame base, yaw, roll, pitch, Lcam, Rcam;
    Eigen::MatrixXd DH = Eigen::MatrixXd::Identity(4,4);
    double pi = 3.1415926;
};