#include "mjpc/tasks/humanoid/tracking2/tracking2.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <string>
#include <tuple>
#include <iostream>
#include <vector>

#include <mujoco/mujoco.h>
#include "mjpc/utilities.h"
#include <math.h> 
#include <cmath>
#include "tinyxml2.h"

#define PI 3.14159265

namespace {
// compute interpolation between mocap frames
std::tuple<int, int, double, double> ComputeInterpolationValues(double index,
                                                                int max_index) {
  int index_0 = std::floor(std::clamp(index, 0.0, (double)max_index));
  int index_1 = std::min(index_0 + 1, max_index);

  double weight_1 = std::clamp(index, 0.0, (double)max_index) - index_0;
  double weight_0 = 1.0 - weight_1;

  return {index_0, index_1, weight_0, weight_1};
}

// Hardcoded constant matching keyframes from CMU mocap dataset.
constexpr double kFps = 30.0;
std::vector<int> kMotionLengths2;
// constexpr int kMotionLengths[] = {
//     //121,  // Jump - CMU-CMU-02-02_04
//     //14,  // BOX - AABOXWALKFORWARD
//     //84,  // standandjab- standandjab
//     75,  // idle- idle
//     62,  // standfront - standfront
//     61,  // standback - standback
//     28,  // shortstepForwardbox - shortstepForwardbox
//     60,  // kicking - kicking
//     28,  // walkf - walkf
//     84,  // standandjab- standandjab
//     55,  // 26_AA_Boxing_Jab- 26_AA_Boxing_Jab
//     27,  // Boxing_ComboA_Jab_cross- Boxing_ComboA_Jab_cross
//     55,  // 48_AA_Boxing_ComboC_Jab_jab_cross- 48_AA_Boxing_ComboC_Jab_jab_cross
//     28,  // standJump- standJump
//     // 154,  // Kick Spin - CMU-CMU-87-87_01
//     // 115,  // Spin Kick - CMU-CMU-88-88_06
//     // 78,   // Cartwheel (1) - CMU-CMU-88-88_07
//     // 145,  // Crouch Flip - CMU-CMU-88-88_08
//     // 188,  // Cartwheel (2) - CMU-CMU-88-88_09
//     // 260,  // Monkey Flip - CMU-CMU-90-90_19
//     // 279,  // Dance - CMU-CMU-103-103_08
//     // 39,   // Run - CMU-CMU-108-108_13
//     // 510,  // Walk - CMU-CMU-137-137_40
//     // //2066,  // Walk1 - CMU-CMU-137-137_40
//     // 59,  // Walk1 - CMU-CMU-137-137_40
//     // 14,  // Walk1 - CMU-CMU-137-137_40
//     // 14,  // aaboxwalk
//     // 38,  // swordslice
// };

// return length of motion trajectory
//int MotionLength(int id) { return kMotionLengths[id]; }
int MotionLength(int id) { return kMotionLengths2[id]-1; }

// return starting keyframe index for motion
int MotionStartIndex(int id) {
  int start = 0;
  for (int i = 0; i < id; i++) {
    start += MotionLength(i);
  }
  return start;
}

// names for humanoid bodies
const std::array<std::string, 14> body_names = {
    // "pelvis",    "head",   
       "ltoe",  "rtoe", 
        "lheel",  "rheel",
    "lknee",     "rknee",     "lhand", "rhand", "lelbow", "relbow",
    "lshoulder", "rshoulder", "lhip",  "rhip",
};

}  // namespace

namespace mjpc::humanoid {
//double* arr;
double* arr222;
//double arr2[61*10][48];
double arr22[61*100][48];
std::vector<std::string> listofxmlkeyframes;
bool loadedinitialxmls=false;



bool loadXMLlistKeys(const std::string& fileName) 
{
  tinyxml2::XMLDocument doc;
    if (doc.LoadFile(fileName.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Failed to load file: " << fileName << std::endl;
        return false;
    }

    tinyxml2::XMLElement* mujoco = doc.FirstChildElement("mujoco");
    if (!mujoco) {
        std::cerr << "No <mujoco> element found." << std::endl;
        return false;
    }

    tinyxml2::XMLElement* include = mujoco->FirstChildElement("include");
    while (include) {
          std::string str2 = include->Attribute("file");
         //std::string str1 = "D:/engineslist/px/mujoco/24/05/tasksMPC/mujoco_mpcBox/build/mjpc/tasks/humanoid/tracking2";
         std::string str1 = fileName+"/../";
         std::string str3 = str1+str2;
        //listofxmlkeyframes.push_back(include->Attribute("file"));
         listofxmlkeyframes.push_back(str3);
        include = include->NextSiblingElement("include");

    }

  return true;

}

bool loadKeysFromXML2(const std::string& fileName, int startindexanim) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(fileName.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Failed to load file: " << fileName << std::endl;
        return false;
    }

    tinyxml2::XMLElement* mujoco = doc.FirstChildElement("mujoco");
    if (!mujoco) {
        std::cerr << "No <mujoco> element found." << std::endl;
        return false;
    }

    tinyxml2::XMLElement* keyframe = mujoco->FirstChildElement("keyframe");
    if (!keyframe) {
        std::cerr << "No <keyframe> element found." << std::endl;
        return false;
    }

    tinyxml2::XMLElement* keyElement = keyframe->FirstChildElement("key");
    int h=0;
    while (keyElement) {

        std::vector<double> result;
        std::istringstream iss(keyElement->Attribute("mpos"));
        double number;
        while (iss >> number) {
            result.push_back(number);
        }
        //int ScaleCustom=1.0;

        for (int i = 0; i < result.size(); i++)
        {
          //arr22[startindexanim+h][i]=result[i]*ScaleCustom;
          arr22[startindexanim+h][i]=result[i];
        }
        h++;
        keyElement = keyElement->NextSiblingElement("key");
    }
    kMotionLengths2.push_back(h+1);

    return true;
}


std::string Tracking2::XmlPath() const {


    {
      //loadXMLlistKeys("D:/engineslist/px/mujoco/24/05/tasksMPC/mujoco_mpcBox/build/mjpc/tasks/humanoid/tracking2/task2list.xml") ;
      loadXMLlistKeys(GetModelPath("humanoid/tracking2/task2list.xml")) ;

      for (int i = 0; i < listofxmlkeyframes.size(); i++)
      { 
        //int j=i-1;
        loadKeysFromXML2(listofxmlkeyframes[i], MotionStartIndex(i));
      }
      
      arr222=(double*)arr22;
      listofxmlkeyframes.clear();
    }
  return GetModelPath("humanoid/tracking2/task2.xml");
}
std::string Tracking2::Name() const { return "HumTrack2 "; }

// ------------- Residuals for humanoid tracking2 task -------------
//   Number of residuals:
//     Residual (0): Joint vel: minimise joint velocity
//     Residual (1): Control: minimise control
//     Residual (2-11): Tracking position: minimise tracking position error
//         for {root, head, toe, heel, knee, hand, elbow, shoulder, hip}.
//     Residual (11-20): Tracking velocity: minimise tracking velocity error
//         for {root, head, toe, heel, knee, hand, elbow, shoulder, hip}.
//   Number of parameters: 0
// ----------------------------------------------------------------
//double RootPosX,RootPosY,prev_index;
double prev_index=0.0;
void Tracking2::ResidualFn2::Residual(const mjModel *model, const mjData *data,
                                  double *residual) const {
  // ----- get mocap frames ----- //



  
  // get motion start index
  int start = MotionStartIndex(current_mode_);
  // get motion trajectory length
  int length = MotionLength(current_mode_);


  double t=std::fmod(data->time+data->userdata[3],length/kFps);
  double current_index = (t ) * kFps + start;
  int last_key_index = start + length - 1;
  if (current_mode_==1||current_mode_==2)
  {  
    if (data->userdata[2]>current_index)
    //if (current_index>=last_key_index-1)
    {
      current_index=last_key_index-1;
    }
  }
  

  // Positions:
  // We interpolate linearly between two consecutive key frames in order to
  // provide smoother signal for tracking.
  int key_index_0, key_index_1;
  double weight_0, weight_1;
  std::tie(key_index_0, key_index_1, weight_0, weight_1) =
      ComputeInterpolationValues(current_index, last_key_index);


  

  // ----- residual ----- //
  int counter = 0;


  // ----- action ----- //
  mju_copy(&residual[counter], data->ctrl, model->nu);
  counter += model->nu;

  // ----- position ----- //
  // Compute interpolated frame.
  auto get_body_mpos = [&](const std::string &body_name, double result[3]) {
    std::string mocap_body_name = "mocap[" + body_name + "]";
    int mocap_body_id = mj_name2id(model, mjOBJ_BODY, mocap_body_name.c_str());
    assert(0 <= mocap_body_id);
    int body_mocapid = model->body_mocapid[mocap_body_id];
    assert(0 <= body_mocapid);

    // current frame
    mju_scl3(
        result,
        //model->key_mpos + model->nmocap * 3 * key_index_0 + 3 * body_mocapid,
        arr222 + model->nmocap * 3 * key_index_0 + 3 * body_mocapid,
        //arr + model->nmocap * 3 * key_index_0 + 3 * body_mocapid,
        weight_0);

    // next frame
    mju_addToScl3(
        result,
        //model->key_mpos + model->nmocap * 3 * key_index_1 + 3 * body_mocapid,
        arr222 + model->nmocap * 3 * key_index_0 + 3 * body_mocapid,
        //arr + model->nmocap * 3 * key_index_1 + 3 * body_mocapid,
        weight_1);
        
  };

  auto get_body_sensor_pos = [&](const std::string &body_name,
                                 double result[3]) {
    std::string pos_sensor_name = "tracking_pos[" + body_name + "]";
    double *sensor_pos = SensorByName(model, data, pos_sensor_name.c_str());
    mju_copy3(result, sensor_pos);
  };

  // compute marker and sensor averages
  double avg_mpos[3] = {0};
  double avg_sensor_pos[3] = {0};
  int num_body = 0;
  if (key_index_0<data->userdata[2])
  {
    //std::string pos_sensor_name = "tracking_pos[pelvis]";
    //double *sensor_pos = SensorByName(model, data, pos_sensor_name.c_str());

        for (const auto &body_name : body_names) {
      //double body_mpos[3];
      double body_sensor_pos[3];
      //get_body_mpos(body_name, body_mpos);
      //mju_rotVecQuat(body_mpos,  body_mpos, QuatRot);
      //mju_addTo3(body_mpos, avg_MposDelta);
      //mju_addTo3(avg_mpos, body_mpos);
      get_body_sensor_pos(body_name, body_sensor_pos);
      mju_addTo3(avg_sensor_pos, body_sensor_pos);
      num_body++;
    }
      mju_scl3(avg_sensor_pos, avg_sensor_pos, 1.0/num_body);


    data->userdata[0]=avg_sensor_pos[0];
    data->userdata[1]=avg_sensor_pos[1];



  }
  double avg_MposDelta[3] = {data->userdata[0],data->userdata[1],0};
  
  double QuatRot[4] ={1.0,0.0,0.0,0.0};
  const double Axis[3] ={0.0,0.0,1.0};
  mju_axisAngle2Quat(QuatRot, Axis, parameters_[2]*PI/180.0);

  for (const auto &body_name : body_names) {
    double body_mpos[3];
    double body_sensor_pos[3];
    get_body_mpos(body_name, body_mpos);
    mju_rotVecQuat(body_mpos,  body_mpos, QuatRot);
    mju_addTo3(body_mpos, avg_MposDelta);
    mju_addTo3(avg_mpos, body_mpos);
    get_body_sensor_pos(body_name, body_sensor_pos);
    mju_addTo3(avg_sensor_pos, body_sensor_pos);
    num_body++;
  }
  mju_scl3(avg_mpos, avg_mpos, 1.0/num_body);
  mju_scl3(avg_sensor_pos, avg_sensor_pos, 1.0/num_body);

  // residual for averages
  mju_sub3(&residual[counter], avg_mpos, avg_sensor_pos);
  counter += 3;

  for (const auto &body_name : body_names) {
    double body_mpos[3];
    get_body_mpos(body_name, body_mpos);
    mju_rotVecQuat(body_mpos,  body_mpos, QuatRot);
    mju_addTo3(body_mpos, avg_MposDelta);

    // current position
    double body_sensor_pos[3];
    get_body_sensor_pos(body_name, body_sensor_pos);

    mju_subFrom3(body_mpos, avg_mpos);
    mju_subFrom3(body_sensor_pos, avg_sensor_pos);

    mju_sub3(&residual[counter], body_mpos, body_sensor_pos);

    counter += 3;
  }

  // data->userdata[0]=avg_sensor_pos[0];
  // data->userdata[1]=avg_sensor_pos[1];

  // data->userdata[2]=(double)key_index_0;
  // if (data->userdata[2]<current_index)
  // {
  //   //data->userdata[2]=(double)key_index_0;
  //   //data->userdata[2]=(double)current_index;

  // }


  CheckSensorDim(model, counter);
}

// --------------------- Transition for humanoid task -------------------------
//   Set `data->mocap_pos` based on `data->time` to move the mocap sites.
//   Linearly interpolate between two consecutive key frames in order to
//   smooth the transitions between keyframes.
// ----------------------------------------------------------------------------
//double conCost;
//MARK: Transition Locked
double timerCountWalk;
void Tracking2::TransitionLocked(mjModel *model, mjData *d) {
  

  // get motion start index
  int start = MotionStartIndex(mode);
  // get motion trajectory length
  int length = MotionLength(mode);


    // check for motion switch
  if (residual_.current_mode_ != mode || d->time == 0.0) {
    residual_.current_mode_ = mode;       // set motion id
    residual_.reference_time_ = d->time;  // set reference time
    d->userdata[3]=10.000;

  }

  double t=std::fmod(d->time+d->userdata[3],length/ kFps);
  double current_index = (t) * kFps + start;
  int last_key_index = start + length - 1;
  //end legacy

  // Positions:
  // We interpolate linearly between two consecutive key frames in order to
  // provide smoother signal for tracking.
  int key_index_0, key_index_1;
  double weight_0, weight_1;
  std::tie(key_index_0, key_index_1, weight_0, weight_1) =
      ComputeInterpolationValues(current_index, last_key_index);

// ----- position ----- //
  // Compute interpolated frame.
  // auto get_body_mpos = [&](const std::string &body_name, double result[3]) {
  //   std::string mocap_body_name = "mocap[" + body_name + "]";
  //   int mocap_body_id = mj_name2id(model, mjOBJ_BODY, mocap_body_name.c_str());
  //   assert(0 <= mocap_body_id);
  //   int body_mocapid = model->body_mocapid[mocap_body_id];
  //   assert(0 <= body_mocapid);

  //   // current frame
  //   mju_scl3(
  //       result,
  //       //model->key_mpos + model->nmocap * 3 * key_index_0 + 3 * body_mocapid,
  //       arr222 + model->nmocap * 3 * key_index_0 + 3 * body_mocapid,
  //       //arr + model->nmocap * 3 * key_index_0 + 3 * body_mocapid,
  //       weight_0);

  //   // next frame
  //   mju_addToScl3(
  //       result,
  //       //model->key_mpos + model->nmocap * 3 * key_index_1 + 3 * body_mocapid,
  //       arr222 + model->nmocap * 3 * key_index_0 + 3 * body_mocapid,
  //       //arr + model->nmocap * 3 * key_index_1 + 3 * body_mocapid,
  //       weight_1);
        
  // };

  auto get_body_sensor_pos = [&](const std::string &body_name,
                                 double result[3]) {
    std::string pos_sensor_name = "tracking_pos[" + body_name + "]";
    double *sensor_pos = SensorByName(model, d, pos_sensor_name.c_str());
    mju_copy3(result, sensor_pos);
  };
    int num_body = 0;
    if (timerCountWalk>0.50&&mode==12)    
    {
      timerCountWalk=0.0;
      std::string pos_sensor_name = "tracking_pos[pelvis]";
      double *sensor_pos = SensorByName(model, d, pos_sensor_name.c_str());
      double mpospelvis[3];
      mpospelvis[0]=d->mocap_pos[0];
      mpospelvis[1]=d->mocap_pos[1];
      mpospelvis[2]=d->mocap_pos[2];
      mju_sub3(mpospelvis,sensor_pos,mpospelvis);
      if (mju_normalize3(mpospelvis)>0.5)
      {
        d->userdata[0]=sensor_pos[0];
        d->userdata[1]=sensor_pos[1];
        d->userdata[3]=-d->time;

      }
      

    }else
    {
      timerCountWalk+=model->opt.timestep;
    }

    if (key_index_0<d->userdata[2])
    {

      // std::string pos_sensor_name = "tracking_pos[pelvis]";
      // double *sensor_pos = SensorByName(model, d, pos_sensor_name.c_str());
        double avg_sensor_pos[3] = {0};

      for (const auto &body_name : body_names) {
      double body_sensor_pos[3];
      get_body_sensor_pos(body_name, body_sensor_pos);
      mju_addTo3(avg_sensor_pos, body_sensor_pos);
      num_body++;
      }
      mju_scl3(avg_sensor_pos, avg_sensor_pos, 1.0/num_body);

      d->userdata[0]=avg_sensor_pos[0];
      d->userdata[1]=avg_sensor_pos[1];
      if (residual_.CostValue(d->sensordata)<250)
      {
        if (residual_.current_mode_==1||residual_.current_mode_==2)
        {
          residual_.current_mode_=d->userdata[4];
          mode=d->userdata[4];
          d->userdata[3]=-d->time;
        }
      }
    }
    
    
  mj_markStack(d);

  mjtNum *mocap_pos_0 = mj_stackAllocNum(d, 3 * model->nmocap);
  mjtNum *mocap_pos_1 = mj_stackAllocNum(d, 3 * model->nmocap);

  // Compute interpolated frame.
  mju_scl(mocap_pos_0, 
  arr222 + model->nmocap * 3 * key_index_0,
          weight_0, model->nmocap * 3);

  mju_scl(mocap_pos_1, 
  arr222 + model->nmocap * 3 * key_index_1,
          weight_1, model->nmocap * 3);

  mju_copy(d->mocap_pos, mocap_pos_0, model->nmocap * 3);
  mju_addTo(d->mocap_pos, mocap_pos_1, model->nmocap * 3);


  double QuatRot[4] ={1.0,0.0,0.0,0.0};
  const double Axis[3] ={0.0,0.0,1.0};
  double body_mposM[3] ={0.0,0.0,0.0};
  mju_axisAngle2Quat(QuatRot, Axis, parameters[2]*PI/180.0);


  for (size_t i = 0; i < model->nmocap*3; i++)
  {
    body_mposM[0]=d->mocap_pos[i*3+0];
    body_mposM[1]=d->mocap_pos[i*3+1];
    body_mposM[2]=d->mocap_pos[i*3+2];
    mju_rotVecQuat(body_mposM,  body_mposM, QuatRot);
    d->mocap_pos[i*3+0]=body_mposM[0]+d->userdata[0];
    d->mocap_pos[i*3+1]=body_mposM[1]+d->userdata[1];
    d->mocap_pos[i*3+2]=body_mposM[2];
  }


    std::string pos_sensor_name = "tracking_pos[pelvis]";
    double *sensor_pos = SensorByName(model, d, pos_sensor_name.c_str());

  
    if (sensor_pos[2]<0.6)
    {
      //if (residual_.current_mode_ ==0&&d->sensordata[0] )
      //if (residual_.current_mode_ ==0&&residual_.CostValue(d->sensordata)>150 &&t>1.0)
      if (residual_.current_mode_ !=1&&residual_.current_mode_ !=2&&residual_.CostValue(d->sensordata)>150)
      {
        

        int joint_id = mj_name2id(model, mjOBJ_JOINT, "root");
        //int joint_id = mj_name2id(model, mjOBJ_JOINT, "abdomen_x");
        if (joint_id == -1) {
            std::cerr << "Joint not found." << std::endl;
            //return 1;
        }
        // Free joint quaternion index in qpos
        int qposadr = model->jnt_qposadr[joint_id];
        //const mjtNum* quat = d->qpos + qposadr + 3; // Skip position (3 values)
        mjtNum res[9];
        //mju_quat2Mat( res, qposadr);
        mju_quat2Mat( res, d->qpos + qposadr + 3);
        //std::cout<<res[0]<<std::endl;
        if (res[6]<0.0)
        {
          d->userdata[4]=residual_.current_mode_;
          residual_.current_mode_ = 1; 
          mode=1;
          d->userdata[0]=sensor_pos[0];
          d->userdata[1]=sensor_pos[1];
          d->userdata[3]=-d->time;


          std::string pos_sensor_name = "tracking_pos[head]";
          double *sensor_pos1 = SensorByName(model, d, pos_sensor_name.c_str());
          double dir[2]={sensor_pos1[0]-sensor_pos[0],sensor_pos1[1]-sensor_pos[1]};
          parameters[2]=std::atan2(dir[1], dir[0])* (180.0 / PI);
          
        }else if (res[6]>0.0)
        {
          d->userdata[4]=residual_.current_mode_;
          residual_.current_mode_ = 2; 
          mode=2;
          d->userdata[3]=-d->time;
          d->userdata[0]=sensor_pos[0];
          d->userdata[1]=sensor_pos[1];

          std::string pos_sensor_name = "tracking_pos[head]";
          double *sensor_pos1 = SensorByName(model, d, pos_sensor_name.c_str());
          double dir[2]={-sensor_pos1[0]+sensor_pos[0],-sensor_pos1[1]+sensor_pos[1]};
          parameters[2]=std::atan2(dir[1], dir[0])* (180.0 / PI);

          
          
        }
        

      }
    }

  d->userdata[2]=(double)key_index_0;
  mj_freeStack(d);
}


void Tracking2::InputDataFunc(const mjModel* model, mjData* data)
{
  DataVectorInput.clear();

  int length = MotionLength(residual_.current_mode_);
  //// legacy
  double t=std::fmod(data->time,length/kFps);
  DataVectorInput.push_back(t);


}
std::string mjpc::humanoid::Tracking2::XmlPath2() const
{
      return GetModelPath("humanoid/tracking2/task22.xml");

}

}  // namespace mjpc::humanoid
