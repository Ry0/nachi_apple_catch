#include "draw_arms.h"

//using namespace std;
//using namespace ode_utils;

extern dWorldID      world;                   // 動力学計算用のワールド
extern dSpaceID      space;                   // 衝突検出用のスペース
extern dGeomID       ground;                  // 地面のジオメトリID番号
extern dJointGroupID contactgroup;            // 接触点グループ
extern dJointID      joint[7];              // ジョイントのID番号
extern dJointID      bodyjoint1[8], bodyjoint2[13], bodyjoint3, bodyjoint4[4], bodyjoint5, bodyjoint6[4];
extern dBodyID       sensor;                  // センサ用のボディID
extern dJointID      sensor_joint;            // センサ固定用の関節
extern dsFunctions   fn;                      // ドロースタッフの描画関数

extern MyObject base, bodyparts1[9], bodyparts2[14], bodyparts3[2], bodyparts4[5], bodyparts5[2], bodyparts6[5];
extern dReal  THETA[7];             // 関節の目標角度[rad]


extern dJointID box1joint[5], box2joint[5];
extern MyObject box1parts[5], box2parts[5];

extern int  ANSWER;              // 逆運動学の解
extern int  i,j;

extern dReal P[3];             // 先端の位置

// 有顔ベクトル(a,b)
extern dReal a[3];//?わっからーん
extern dReal b[3];//?わっからーん
extern dReal T[2];
extern dReal l[7];   // リンクの長さ[m]



void makeBox1()
{
  dMass mass;                                    // 質量パラメータ
  #define BOX1_X 0.3
  #define BOX1_Y 0.3

  dReal  box1_mass[5]  = {0.02, 0.01, 0.01, 0.01, 0.01};// 質量
  dReal  box1_x_length[5] = {0.20, 0.01, 0.01, 0.18, 0.18};
  dReal  box1_y_length[5] = {0.20, 0.20, 0.20, 0.01, 0.01};
  dReal  box1_z_length[5] = {0.01, 0.10, 0.10, 0.10, 0.10};

  dReal  box1_start_x[5] = {BOX1_X+0, BOX1_X+0.095, BOX1_X-0.095, BOX1_X+0, BOX1_X+0};// 重心 y
  dReal  box1_start_y[5] = {BOX1_Y+0, BOX1_Y+0, BOX1_Y+0, BOX1_Y+0.095, BOX1_Y-0.095};// 重心 y
  dReal  box1_start_z[5] = {0.005, 0.06, 0.06, 0.06, 0.06};// 重心 z
  dReal  box1_hinge_x[4] = {BOX1_X+0.095, BOX1_X-0.095, 0.0, 0.0};
  dReal  box1_hinge_y[4] = {0.0, 0.0, BOX1_Y+0.095, BOX1_Y-0.095};
  dReal  box1_hinge_z[4] = {0.01, 0.01, 0.01, 0.01};


  //パーツの作成
  for (int i = 0; i < 5; i++) {
    box1parts[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass, box1_mass[i], box1_x_length[i], box1_y_length[i], box1_z_length[i]);
    dBodySetMass(box1parts[i].body, &mass);

    box1parts[i].geom = dCreateBox(space, box1_x_length[i], box1_y_length[i], box1_z_length[i]);
    dGeomSetBody(box1parts[i].geom, box1parts[i].body);
    dBodySetPosition(box1parts[i].body, box1_start_x[i], box1_start_y[i], box1_start_z[i]);
  }


  //パーツの合体
  box1joint[0] = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(box1joint[0], box1parts[0].body, 0);
  dJointSetFixed(box1joint[0]);

  for (int i = 1; i < 5; i++) {
    box1joint[i] = dJointCreateHinge(world,0);
    dJointAttach(box1joint[i], box1parts[0].body, box1parts[i].body);
    dJointSetHingeAxis(box1joint[i],0, 0, 1);
    dJointSetHingeAnchor(box1joint[i], box1_hinge_x[i-1], box1_hinge_y[i-1], box1_hinge_z[i-1]);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(box1joint[i], dParamLoStop, 0);
    dJointSetHingeParam(box1joint[i], dParamHiStop, 0);
  }
}



void drawBox1()
{
  dReal box_length0[] = {0.20, 0.20, 0.01};
  dReal box_length12[] = {0.01, 0.20, 0.10};
  dReal box_length34[] = {0.18, 0.01, 0.10};

  double R = 0.0/255.0;
  double G = 191.0/255.0;
  double B = 255.0/255.0;
  //dsSetColor(R,G,B);
  dsSetColorAlpha(R, G, B, 0.5);


  dsDrawBox(dBodyGetPosition(box1parts[0].body), dBodyGetRotation(box1parts[0].body), box_length0);
  dsDrawBox(dBodyGetPosition(box1parts[1].body), dBodyGetRotation(box1parts[1].body), box_length12);
  dsDrawBox(dBodyGetPosition(box1parts[2].body), dBodyGetRotation(box1parts[2].body), box_length12);
  dsDrawBox(dBodyGetPosition(box1parts[3].body), dBodyGetRotation(box1parts[3].body), box_length34);
  dsDrawBox(dBodyGetPosition(box1parts[4].body), dBodyGetRotation(box1parts[4].body), box_length34);

}


void makeBox2()
{
  dMass mass;                                    // 質量パラメータ
  #define BOX2_X 0.3
  #define BOX2_Y -0.3

  dReal  box2_mass[5]  = {0.02, 0.01, 0.01, 0.01, 0.01};// 質量
  dReal  box2_x_length[5] = {0.20, 0.01, 0.01, 0.18, 0.18};
  dReal  box2_y_length[5] = {0.20, 0.20, 0.20, 0.01, 0.01};
  dReal  box2_z_length[5] = {0.01, 0.10, 0.10, 0.10, 0.10};

  dReal  box2_start_x[5] = {BOX2_X+0, BOX2_X+0.095, BOX2_X-0.095, BOX2_X+0, BOX2_X+0};// 重心 y
  dReal  box2_start_y[5] = {BOX2_Y+0, BOX2_Y+0, BOX2_Y+0, BOX2_Y+0.095, BOX2_Y-0.095};// 重心 y
  dReal  box2_start_z[5] = {0.005, 0.06, 0.06, 0.06, 0.06};// 重心 z
  dReal  box2_hinge_x[4] = {BOX2_X+0.095, BOX2_X-0.095, 0.0, 0.0};
  dReal  box2_hinge_y[4] = {0.0, 0.0, BOX2_Y+0.095, BOX2_Y-0.095};
  dReal  box2_hinge_z[4] = {0.01, 0.01, 0.01, 0.01};


  //パーツの作成
  for (int i = 0; i < 5; i++) {
    box2parts[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass, box2_mass[i], box2_x_length[i], box2_y_length[i], box2_z_length[i]);
    dBodySetMass(box1parts[i].body, &mass);

    box2parts[i].geom = dCreateBox(space, box2_x_length[i], box2_y_length[i], box2_z_length[i]);
    dGeomSetBody(box2parts[i].geom, box2parts[i].body);
    dBodySetPosition(box2parts[i].body, box2_start_x[i], box2_start_y[i], box2_start_z[i]);
  }


  //パーツの合体
  box2joint[0] = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(box2joint[0], box2parts[0].body, 0);
  dJointSetFixed(box2joint[0]);

  for (int i = 1; i < 5; i++) {
    box2joint[i] = dJointCreateHinge(world,0);
    dJointAttach(box2joint[i], box2parts[0].body, box2parts[i].body);
    dJointSetHingeAxis(box2joint[i],0, 0, 1);
    dJointSetHingeAnchor(box2joint[i], box2_hinge_x[i-1], box2_hinge_y[i-1], box2_hinge_z[i-1]);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(box2joint[i], dParamLoStop, 0);
    dJointSetHingeParam(box2joint[i], dParamHiStop, 0);
  }
}



void drawBox2()
{
  dReal box_length0[] = {0.20, 0.20, 0.01};
  dReal box_length12[] = {0.01, 0.20, 0.10};
  dReal box_length34[] = {0.18, 0.01, 0.10};

  double R = 0.0/255.0;
  double G = 191.0/255.0;
  double B = 255.0/255.0;
  //dsSetColor(R,G,B);
  dsSetColorAlpha(R, G, B, 0.5);


  dsDrawBox(dBodyGetPosition(box2parts[0].body), dBodyGetRotation(box2parts[0].body), box_length0);
  dsDrawBox(dBodyGetPosition(box2parts[1].body), dBodyGetRotation(box2parts[1].body), box_length12);
  dsDrawBox(dBodyGetPosition(box2parts[2].body), dBodyGetRotation(box2parts[2].body), box_length12);
  dsDrawBox(dBodyGetPosition(box2parts[3].body), dBodyGetRotation(box2parts[3].body), box_length34);
  dsDrawBox(dBodyGetPosition(box2parts[4].body), dBodyGetRotation(box2parts[4].body), box_length34);

}

void makeSensor()
{
  dMass mass;
  double sx = 0.0, sy = 0.0, sz = 1.268;  // センサの初期座標[m]
  double r = 0.04, weight = 0.01; // センサのサイズ[m]と重量[kg]

  sensor = dBodyCreate(world);          // センサの生成
  dBodySetPosition(sensor,sx,sy,sz);
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass, weight, r);
  dBodySetMass(sensor, &mass);

  sensor_joint = dJointCreateFixed(world, 0); // 固定ジョイントの生成
  dJointAttach(sensor_joint, bodyparts6[3].body, sensor); // 先端リンクと結合
  dJointSetFixed(sensor_joint);
}


// センサ位置の表示
void printSensorPosition()
{
  double *pos = (double *) dBodyGetPosition(sensor);
  double pos2[3];

  for (int i = 0; i < 3; ++i){//擬似的にセンサーの値をジャミンググリッパの先端の座標に持ってくる
    pos2[i] = pos[i]+a[i]*0.04;
  }

  //printf("Current Position: x=%6.3f y=%6.3f z=%6.3f \n", pos2[0], pos2[1], pos2[2]);
  printf("Current Position: x=%lf y=%lf z=%lf \n", pos2[0], pos2[1], pos2[2]);
}


void makeBase()
{
  dMass mass;                                    // 質量パラメータ
  //ベースの生成
  base.body = dBodyCreate(world);
  dBodySetPosition(base.body, 0.0, 0.0, 0.10);
  dMassSetZero(&mass);
  dMassSetCapsuleTotal(&mass,9.0, 3, 0.12, 0.2);
  dBodySetMass(base.body, &mass);
  base.geom = dCreateCapsule(space, 0.12, 0.2);
  dGeomSetBody(base.geom, base.body);

  // ジョイントの生成とリンクへの取り付け
  joint[0] = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(joint[0], base.body, 0);
  dJointSetFixed(joint[0]);
}

void  makeArm()
{
  dMass mass;                                    // 質量パラメータ
  dMatrix3 R;

  dReal  parts1_mass[9]  = {0.04, 0.04, 0.2, 0.04, 0.04, 0.01, 0.01, 0.01, 0.01};// 質量
  dReal  parts1_x_length[5] = {0.15, 0.15, 0.15, 0.15, 0.15};
  dReal  parts1_y_length[5] = {0.02, 0.03, 0.1, 0.03, 0.02};
  dReal  parts1_z_length[5] = {0.12, 0.12, 0.07, 0.12, 0.12};
  dReal  parts1_r = 0.075;
  dReal  parts1_l[4] = {0.02, 0.03, 0.03, 0.02};
  dReal  parts1_start_x = 0.0;// 重心 x
  dReal  parts1_start_y[9] = {-0.09, -0.065, 0, 0.065, 0.09, -0.09, -0.065, 0.065, 0.09};// 重心 y
  dReal  parts1_start_z[9] = {0.26, 0.26, 0.235, 0.26, 0.26, 0.32, 0.32, 0.32, 0.32};// 重心 z
  dReal  parts1_hinge_y[4] = {-0.08, -0.05, 0.05, 0.08};

  dReal  parts2_mass[14]  = {0.05, 0.05, 0.35, 0.05, 0.05, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.05};// 質量
  dReal  parts2_x_length[5] = {0.1};
  dReal  parts2_y_length[5] = {0.02, 0.03, 0.1, 0.03, 0.02};
  dReal  parts2_z_length[5] = {0.17, 0.17, 0.21, 0.17, 0.17};
  dReal  parts2_r = 0.05;
  dReal  parts2_l[9] = {0.02, 0.03, 0.03, 0.02, 0.02, 0.03, 0.03, 0.02, 0.1};
  dReal  parts2_start_x = 0.0;// 重心 x
  dReal  parts2_start_y[14] = {-0.09, -0.065, 0, 0.065, 0.09, -0.09, -0.065, 0.065, 0.09, -0.09, -0.065, 0.065, 0.09, 0};// 重心 y
  dReal  parts2_start_z[14] = {0.59, 0.59, 0.45, 0.59, 0.59, 0.675, 0.675, 0.675, 0.675, 0.505, 0.505, 0.505, 0.505, 0.345};// 重心 z
  dReal  parts2_hinge_y[4] = {-0.08, -0.05, 0.05, 0.08};

  dReal  parts3_mass[2]  = {0.18, 0.02};// 質量
  dReal  parts3_x_length[2] = {0.1, 0.02};
  dReal  parts3_y_length[2] = {0.1, 0.1};
  dReal  parts3_z_length[2] = {0.2, 0.18};
  dReal  parts3_start_x[2] = {0, 0.06};// 重心 x
  dReal  parts3_start_y = 0;// 重心 y
  dReal  parts3_start_z[2] = {0.725, 0.715};// 重心 z

  dReal  parts4_mass[5]  = {0.05, 0.15, 0.4, 0.15, 0.05};// 質量
  dReal  parts4_x_length[3] = {0.12, 0.12, 0.12};
  dReal  parts4_y_length[3] = {0.02, 0.08, 0.02};
  dReal  parts4_z_length[3] = {0.19, 0.14, 0.19};
  dReal  parts4_start_x = 0;// 重心 x
  dReal  parts4_start_y[5] = {-0.05, 0, 0.05, -0.05, 0.05};// 重心 y
  dReal  parts4_start_z[5] = {0.92, 0.895, 0.92, 1.015, 1.015};// 重心 z
  dReal  parts4_hinge_y[2] = {-0.04, 0.04};

  dReal  parts5_mass[2]  = {0.08, 0.02};// 質量
  dReal  parts5_x_length = 0.08;
  dReal  parts5_y_length = 0.08;
  dReal  parts5_z_length = 0.073;
  dReal  parts5_start_x = 0;// 重心 x
  dReal  parts5_start_y = 0;// 重心 y
  dReal  parts5_start_z[2] = {1.0515, 1.015};// 重心 z

  dReal  parts6_mass[5]  = {0.02, 0.18, 0.1, 0.2, 0.2};// 質量
  dReal  parts6_start_x[5] = {0, 0, 0, 0, 0};// 重心 x
  dReal  parts6_start_y[5] = {0, 0, 0, 0, 0};// 重心 y
  dReal  parts6_start_z[5] = {1.093, 1.118, 1.153, 1.198, 1.248};// 重心 z
  dReal  parts6_z_length[5] = {0.01, 0.04, 0.03, 0.06, 0.04};  // 長さ
  dReal  parts6_r[5]      = {0.04, 0.038, 0.02, 0.04, 0.05};  // 半径
  dReal  parts6_hinge_z[4] = {1.098, 1.138, 1.168, 1.228};

  dRFromAxisAndAngle(R, 1, 0, 0, M_PI/2.0);

  //パーツ1の作成
  for (int i = 0; i < 5; i++) {
    bodyparts1[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,parts1_mass[i], parts1_x_length[i], parts1_y_length[i], parts1_z_length[i]);
    dBodySetMass(bodyparts1[i].body, &mass);

    bodyparts1[i].geom = dCreateBox(space, parts1_x_length[i], parts1_y_length[i], parts1_z_length[i]);
    dGeomSetBody(bodyparts1[i].geom, bodyparts1[i].body);
    dBodySetPosition(bodyparts1[i].body, parts1_start_x, parts1_start_y[i], parts1_start_z[i]);
  }

  for (int i = 5; i < 9; i++) {
    bodyparts1[i].body = dBodyCreate(world);
    dBodySetPosition(bodyparts1[i].body, parts1_start_x, parts1_start_y[i], parts1_start_z[i]);
    dMassSetZero(&mass);
    dMassSetCylinderTotal(&mass, parts1_mass[i], 2, parts1_r, parts1_l[i-5]);
    dBodySetMass(bodyparts1[i].body, &mass);

    bodyparts1[i].geom = dCreateCylinder(space, parts1_r, parts1_l[i-5]);
    dGeomSetBody(bodyparts1[i].geom, bodyparts1[i].body);
    dBodySetRotation(bodyparts1[i].body, R);
  }

  //パーツ1の合体
  for (int i = 0; i < 4; i++) {
    bodyjoint1[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint1[i], bodyparts1[i].body, bodyparts1[i+1].body);
    dJointSetHingeAxis(bodyjoint1[0],0, 1, 0);
    dJointSetHingeAnchor(bodyjoint1[i], 0, parts1_hinge_y[i], 0.235);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint1[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint1[i],dParamHiStop, 0);
  }
  for (int i = 4; i < 6; i++) {
    bodyjoint1[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint1[i], bodyparts1[i+1].body, bodyparts1[i-4].body);
    dJointSetHingeAxis(bodyjoint1[0],0, 0, 1);
    dJointSetHingeAnchor(bodyjoint1[i], 0, parts1_start_y[i-4], 0.32);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint1[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint1[i],dParamHiStop, 0);
  }
  for (int i = 6; i < 8; i++) {
    bodyjoint1[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint1[i], bodyparts1[i+1].body, bodyparts1[i-3].body);
    dJointSetHingeAxis(bodyjoint1[0],0, 0, 1);
    dJointSetHingeAnchor(bodyjoint1[i], 0, parts1_start_y[i-3], 0.32);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint1[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint1[i],dParamHiStop, 0);
  }


  //パーツ2の作成
  for (int i = 0; i < 5; i++) {
    bodyparts2[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,parts2_mass[i], parts2_x_length[i], parts2_y_length[i], parts2_z_length[i]);
    dBodySetMass(bodyparts1[i].body, &mass);

    bodyparts2[i].geom = dCreateBox(space, parts2_x_length[i], parts2_y_length[i], parts2_z_length[i]);
    dGeomSetBody(bodyparts2[i].geom, bodyparts2[i].body);
    dBodySetPosition(bodyparts2[i].body, parts2_start_x, parts2_start_y[i], parts2_start_z[i]);
  }

  for (int i = 5; i < 14; i++) {
    bodyparts2[i].body = dBodyCreate(world);
    dBodySetPosition(bodyparts2[i].body, parts2_start_x, parts2_start_y[i], parts2_start_z[i]);
    dMassSetZero(&mass);
    dMassSetCylinderTotal(&mass, parts2_mass[i], 2, parts2_r, parts2_l[i-5]);
    dBodySetMass(bodyparts2[i].body, &mass);

    bodyparts2[i].geom = dCreateCylinder(space, parts2_r, parts2_l[i-5]);
    dGeomSetBody(bodyparts2[i].geom, bodyparts2[i].body);
    dBodySetRotation(bodyparts2[i].body, R);
  }

  //パーツ2の合体
  for (int i = 0; i < 4; i++) {
    bodyjoint2[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint2[i], bodyparts2[i].body, bodyparts2[i+1].body);
    dJointSetHingeAxis(bodyjoint2[0],0, 1, 0);
    dJointSetHingeAnchor(bodyjoint2[i], 0, parts2_hinge_y[i], 0.755);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint2[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint2[i],dParamHiStop, 0);
  }
  for (int i = 4; i < 6; i++) {
    bodyjoint2[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint2[i], bodyparts2[i+1].body, bodyparts2[i-4].body);
    dJointSetHingeAxis(bodyjoint2[0],0, 0, 1);
    dJointSetHingeAnchor(bodyjoint2[i], 0, parts2_start_y[i-4], 0.675);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint2[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint2[i],dParamHiStop, 0);
  }
  for (int i = 6; i < 8; i++) {
    bodyjoint2[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint2[i], bodyparts2[i+1].body, bodyparts2[i-3].body);
    dJointSetHingeAxis(bodyjoint2[0],0, 0, 1);
    dJointSetHingeAnchor(bodyjoint2[i], 0, parts2_start_y[i-3], 0.675);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint2[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint2[i],dParamHiStop, 0);
  }
  for (int i = 8; i < 10; i++) {
    bodyjoint2[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint2[i], bodyparts2[i+1].body, bodyparts2[i-8].body);
    dJointSetHingeAxis(bodyjoint2[0],0, 0, 1);
    dJointSetHingeAnchor(bodyjoint2[i], 0, parts2_start_y[i-8], 0.505);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint2[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint2[i],dParamHiStop, 0);
  }
  for (int i = 10; i < 12; i++) {
    bodyjoint2[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint2[i], bodyparts2[i+1].body, bodyparts2[i-7].body);
    dJointSetHingeAxis(bodyjoint2[0],0, 0, 1);
    dJointSetHingeAnchor(bodyjoint2[i], 0, parts2_start_y[i-7], 0.505);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint2[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint2[i],dParamHiStop, 0);
  }
  bodyjoint2[12] = dJointCreateHinge(world,0);
  dJointAttach(bodyjoint2[12], bodyparts2[13].body, bodyparts2[2].body);
  dJointSetHingeAxis(bodyjoint2[0],0, 0, 1);
  dJointSetHingeAnchor(bodyjoint2[12], 0, 0, 0.345);//ヒンジの中心点(x,y,z)
  dJointSetHingeParam(bodyjoint2[12],dParamLoStop, 0);
  dJointSetHingeParam(bodyjoint2[12],dParamHiStop, 0);


  //パーツ3の作成
  for (int i = 0; i < 2; i++) {
    bodyparts3[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,parts3_mass[i], parts3_x_length[i], parts3_y_length[i], parts3_z_length[i]);
    dBodySetMass(bodyparts3[i].body, &mass);

    bodyparts3[i].geom = dCreateBox(space, parts3_x_length[i], parts3_y_length[i], parts3_z_length[i]);
    dGeomSetBody(bodyparts3[i].geom, bodyparts3[i].body);
    dBodySetPosition(bodyparts3[i].body, parts3_start_x[i], parts3_start_y, parts3_start_z[i]);
  }

  //パーツ3の合体
    bodyjoint3 = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint3, bodyparts3[0].body, bodyparts3[1].body);
    dJointSetHingeAxis(bodyjoint3,1, 0, 0);
    dJointSetHingeAnchor(bodyjoint3, 0, 0.05, 0.715);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint3,dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint3,dParamHiStop, 0);


  //パーツ4の作成
  for (int i = 0; i < 3; i++) {
    bodyparts4[i].body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,parts4_mass[i], parts4_x_length[i], parts4_y_length[i], parts4_z_length[i]);
    dBodySetMass(bodyparts4[i].body, &mass);

    bodyparts4[i].geom = dCreateBox(space, parts4_x_length[i], parts4_y_length[i], parts4_z_length[i]);
    dGeomSetBody(bodyparts4[i].geom, bodyparts4[i].body);
    dBodySetPosition(bodyparts4[i].body, parts4_start_x, parts4_start_y[i], parts4_start_z[i]);
  }
  for (int i = 3; i < 5; i++) {
    bodyparts4[i].body = dBodyCreate(world);
    dBodySetPosition(bodyparts4[i].body, parts4_start_x, parts4_start_y[i], parts4_start_z[i]);
    dMassSetZero(&mass);
    dMassSetCylinderTotal(&mass, parts4_mass[i], 2, 0.06, 0.02);
    dBodySetMass(bodyparts4[i].body, &mass);

    bodyparts4[i].geom = dCreateCylinder(space, 0.06, 0.02);
    dGeomSetBody(bodyparts4[i].geom, bodyparts4[i].body);
    dBodySetRotation(bodyparts4[i].body, R);
  }

  //パーツ4の合体
  for (int i = 0; i < 2; i++) {
    bodyjoint4[i] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint4[i], bodyparts4[i].body, bodyparts4[i+1].body);
    dJointSetHingeAxis(bodyjoint4[0],0, 1, 0);
    dJointSetHingeAnchor(bodyjoint4[i], 0, parts4_hinge_y[i], 0.895);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint4[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint4[i],dParamHiStop, 0);
  }

    bodyjoint4[2] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint4[2], bodyparts4[3].body, bodyparts4[0].body);
    dJointSetHingeAxis(bodyjoint4[2],0, 0, 1);
    dJointSetHingeAnchor(bodyjoint4[2], 0, parts4_start_y[0], 1.015);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint4[2],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint4[2],dParamHiStop, 0);
    bodyjoint4[3] = dJointCreateHinge(world,0);
    dJointAttach(bodyjoint4[3], bodyparts4[4].body, bodyparts4[2].body);
    dJointSetHingeAxis(bodyjoint4[3],0, 0, 1);
    dJointSetHingeAnchor(bodyjoint4[3], 0, parts4_start_y[2], 1.015);//ヒンジの中心点(x,y,z)
    dJointSetHingeParam(bodyjoint4[3],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint4[3],dParamHiStop, 0);

  //パーツ5の作成
  bodyparts5[0].body  = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass, parts5_mass[0], parts5_x_length, parts5_y_length, parts5_z_length);
  dBodySetMass(bodyparts5[0].body, &mass);

  bodyparts5[0].geom = dCreateBox(space, parts5_x_length, parts5_y_length, parts5_z_length);
  dGeomSetBody(bodyparts5[0].geom, bodyparts5[0].body);
  dBodySetPosition(bodyparts5[0].body, parts5_start_x, parts5_start_y, parts5_start_z[0]);

  bodyparts5[1].body = dBodyCreate(world);
  dBodySetPosition(bodyparts5[1].body, parts5_start_x, parts5_start_y, parts5_start_z[1]);
  dMassSetZero(&mass);
  dMassSetCylinderTotal(&mass, parts5_mass[1], 2, 0.04, 0.08);
  dBodySetMass(bodyparts5[1].body, &mass);

  bodyparts5[1].geom = dCreateCylinder(space, 0.04, 0.08);
  dGeomSetBody(bodyparts5[1].geom, bodyparts5[1].body);
  dBodySetRotation(bodyparts5[1].body, R);

  bodyjoint5 = dJointCreateHinge(world,0);
  dJointAttach(bodyjoint5, bodyparts5[0].body, bodyparts5[1].body);
  dJointSetHingeAxis(bodyjoint5,0, 0, 1);
  dJointSetHingeAnchor(bodyjoint5, 0, 0, 1.015);//ヒンジの中心点(x,y,z)
  dJointSetHingeParam(bodyjoint5, dParamLoStop, 0);
  dJointSetHingeParam(bodyjoint5, dParamHiStop, 0);


  //パーツ6の作成
  for (int i = 0; i < 5; i++) {
    bodyparts6[i].body = dBodyCreate(world);
    dBodySetPosition(bodyparts6[i].body, parts6_start_x[i], parts6_start_y[i], parts6_start_z[i]);
    dMassSetZero(&mass);
    dMassSetCapsuleTotal(&mass, parts6_mass[i], 3, parts6_r[i], parts6_z_length[i]);
    dBodySetMass(bodyparts6[i].body, &mass);
    bodyparts6[i].geom = dCreateCapsule(space, parts6_r[i], parts6_z_length[i]);
    dGeomSetBody(bodyparts6[i].geom, bodyparts6[i].body);
  }

//パーツ6の合体
  for (int i = 0; i < 4; i++) {
    bodyjoint6[i] = dJointCreateHinge(world, 0); // ヒンジジョイント
    dJointAttach(bodyjoint6[i], bodyparts6[i].body, bodyparts6[i+1].body);
    dJointSetHingeAnchor(bodyjoint6[i], 0, 0, parts6_hinge_z[i]);
    dJointSetHingeAxis(bodyjoint6[i], 0, 0, 1);
    dJointSetHingeParam(bodyjoint6[i],dParamLoStop, 0);
    dJointSetHingeParam(bodyjoint6[i],dParamHiStop, 0);
  }




  joint[1] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(joint[1], bodyparts1[2].body, base.body);
  dJointSetHingeAnchor(joint[1], 0, 0, 0.2);
  dJointSetHingeAxis(joint[1], 0, 0, 1);

  joint[2] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(joint[2], bodyparts2[13].body, bodyparts1[6].body);
  dJointSetHingeAnchor(joint[2], 0, -0.05, 0.345);
  dJointSetHingeAxis(joint[2], 0, 1, 0);

  joint[3] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(joint[3], bodyparts3[0].body, bodyparts2[6].body);
  dJointSetHingeAnchor(joint[3], 0, -0.05, 0.675);
  dJointSetHingeAxis(joint[3], 0, 1, 0);

  joint[4] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(joint[4], bodyparts4[1].body, bodyparts3[0].body);
  dJointSetHingeAnchor(joint[4], 0, 0, 0.825);
  dJointSetHingeAxis(joint[4], 0, 0, 1);

  joint[5] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(joint[5], bodyparts5[1].body, bodyparts4[3].body);
  dJointSetHingeAnchor(joint[5], 0, -0.04, 1.015);
  dJointSetHingeAxis(joint[5], 0, 1, 0);

  joint[6] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(joint[6], bodyparts6[0].body, bodyparts5[0].body);
  dJointSetHingeAnchor(joint[6], 0, 0, 1.088);
  dJointSetHingeAxis(joint[6], 0, 0, 1);
}
void drawBase()
{
  dReal r,length;
  double R = 0/255;
  double G = 0/255;
  double B = 0/255;

  dsSetColor(R,G,B);
  dGeomCapsuleGetParams(base.geom, &r, &length);
  dsDrawCylinder(dBodyGetPosition(base.body), dBodyGetRotation(base.body), length, r);
}
void drawArmSide()
{
  dReal parts1_length_04[] = {0.15, 0.02, 0.12};

  dReal parts2_length_04[] = {0.10, 0.02, 0.17};

  dReal parts3_length_0[] = {0.1, 0.1, 0.2};
  dReal parts3_length_1[] = {0.02, 0.1, 0.18};

  dReal parts4_length_02[] = {0.12, 0.02, 0.19};
  dReal parts4_length_1[] = {0.12, 0.08, 0.14};

  dReal parts5_length[] = {0.08, 0.08, 0.073};

  double R = 240.0/255.0;
  double G = 248.0/255.0;
  double B = 255.0/255.0;

  dsSetColor(R,G,B);

  dsDrawBox(dBodyGetPosition(bodyparts1[0].body), dBodyGetRotation(bodyparts1[0].body), parts1_length_04);
  dsDrawBox(dBodyGetPosition(bodyparts1[4].body), dBodyGetRotation(bodyparts1[4].body), parts1_length_04);
  dsDrawCylinder(dBodyGetPosition(bodyparts1[5].body),dBodyGetRotation(bodyparts1[5].body), 0.02, 0.075);
  dsDrawCylinder(dBodyGetPosition(bodyparts1[8].body),dBodyGetRotation(bodyparts1[8].body), 0.02, 0.075);

  dsDrawBox(dBodyGetPosition(bodyparts2[0].body), dBodyGetRotation(bodyparts2[0].body), parts2_length_04);
  dsDrawBox(dBodyGetPosition(bodyparts2[4].body), dBodyGetRotation(bodyparts2[4].body), parts2_length_04);
  dsDrawCylinder(dBodyGetPosition(bodyparts2[5].body),dBodyGetRotation(bodyparts2[5].body), 0.02, 0.05);
  dsDrawCylinder(dBodyGetPosition(bodyparts2[8].body),dBodyGetRotation(bodyparts2[8].body), 0.02, 0.05);
  dsDrawCylinder(dBodyGetPosition(bodyparts2[9].body),dBodyGetRotation(bodyparts2[9].body), 0.02, 0.05);
  dsDrawCylinder(dBodyGetPosition(bodyparts2[12].body),dBodyGetRotation(bodyparts2[12].body), 0.02, 0.05);

  dsDrawBox(dBodyGetPosition(bodyparts3[0].body), dBodyGetRotation(bodyparts3[0].body), parts3_length_0);
  dsDrawBox(dBodyGetPosition(bodyparts3[1].body), dBodyGetRotation(bodyparts3[1].body), parts3_length_1);

  dsDrawBox(dBodyGetPosition(bodyparts4[0].body), dBodyGetRotation(bodyparts4[0].body), parts4_length_02);
  dsDrawBox(dBodyGetPosition(bodyparts4[1].body), dBodyGetRotation(bodyparts4[1].body), parts4_length_1);
  dsDrawBox(dBodyGetPosition(bodyparts4[2].body), dBodyGetRotation(bodyparts4[2].body), parts4_length_02);
  dsDrawCylinder(dBodyGetPosition(bodyparts4[3].body),dBodyGetRotation(bodyparts4[3].body), 0.02, 0.06);
  dsDrawCylinder(dBodyGetPosition(bodyparts4[4].body),dBodyGetRotation(bodyparts4[4].body), 0.02, 0.06);

  dsDrawBox(dBodyGetPosition(bodyparts5[0].body), dBodyGetRotation(bodyparts5[0].body), parts5_length);
  dsDrawCylinder(dBodyGetPosition(bodyparts5[1].body),dBodyGetRotation(bodyparts5[1].body), 0.08, 0.04);
}

void drawArmCenter()
{
  dReal parts1_length_13[] = {0.15, 0.03, 0.12};
  dReal parts1_length_2[] = {0.15, 0.1, 0.07};

  dReal parts2_length_13[] = {0.10, 0.03, 0.17};
  dReal parts2_length_2[] = {0.1, 0.1, 0.21};


  double R = 102.0/255.0;
  double G = 102.0/255.0;
  double B = 102.0/255.0;

  dsSetColor(R,G,B);

  dsDrawBox(dBodyGetPosition(bodyparts1[1].body), dBodyGetRotation(bodyparts1[1].body), parts1_length_13);
  dsDrawBox(dBodyGetPosition(bodyparts1[2].body), dBodyGetRotation(bodyparts1[2].body), parts1_length_2);
  dsDrawBox(dBodyGetPosition(bodyparts1[3].body), dBodyGetRotation(bodyparts1[3].body), parts1_length_13);
  dsDrawCylinder(dBodyGetPosition(bodyparts1[6].body),dBodyGetRotation(bodyparts1[6].body), 0.03, 0.075);
  dsDrawCylinder(dBodyGetPosition(bodyparts1[7].body),dBodyGetRotation(bodyparts1[7].body), 0.03, 0.075);

  dsDrawBox(dBodyGetPosition(bodyparts2[1].body), dBodyGetRotation(bodyparts2[1].body), parts2_length_13);
  dsDrawBox(dBodyGetPosition(bodyparts2[2].body), dBodyGetRotation(bodyparts2[2].body), parts2_length_2);
  dsDrawBox(dBodyGetPosition(bodyparts2[3].body), dBodyGetRotation(bodyparts2[3].body), parts2_length_13);
  dsDrawCylinder(dBodyGetPosition(bodyparts2[6].body),dBodyGetRotation(bodyparts2[6].body), 0.03, 0.05);
  dsDrawCylinder(dBodyGetPosition(bodyparts2[7].body),dBodyGetRotation(bodyparts2[7].body), 0.03, 0.05);
  dsDrawCylinder(dBodyGetPosition(bodyparts2[10].body),dBodyGetRotation(bodyparts2[10].body), 0.03, 0.05);
  dsDrawCylinder(dBodyGetPosition(bodyparts2[11].body),dBodyGetRotation(bodyparts2[11].body), 0.03, 0.05);
  dsDrawCylinder(dBodyGetPosition(bodyparts2[13].body),dBodyGetRotation(bodyparts2[13].body), 0.1, 0.05);
}


void drawGripper_start()
{
  dReal parts6_length = 0.01;
  dReal parts6_r = 0.04;

  double R = 0/255.0;
  double G = 0/255.0;
  double B = 0/255.0;

  dsSetColor(R,G,B);

  dsDrawCylinder(dBodyGetPosition(bodyparts6[0].body),dBodyGetRotation(bodyparts6[0].body), parts6_length, parts6_r);
}

void drawGripper()
{
  dReal parts6_length[3] = {0.04, 0.03, 0.06};
  dReal parts6_r[3] = {0.038, 0.02, 0.04};

  double R = 165.0/255.0;
  double G = 165.0/255.0;
  double B = 165.0/255.0;

  dsSetColor(R,G,B);

  dsDrawCylinder(dBodyGetPosition(bodyparts6[1].body),dBodyGetRotation(bodyparts6[1].body), parts6_length[0], parts6_r[0]);
  dsDrawCylinder(dBodyGetPosition(bodyparts6[2].body),dBodyGetRotation(bodyparts6[2].body), parts6_length[1], parts6_r[1]);
  dsDrawCylinder(dBodyGetPosition(bodyparts6[3].body),dBodyGetRotation(bodyparts6[3].body), parts6_length[2], parts6_r[2]);
}

void drawGripper_edge()
{
  dReal parts6_length = 0.04;
  dReal parts6_r = 0.05;

  double R = 15.0/255.0;
  double G = 82.0/255.0;
  double B = 222.0/255.0;

  dsSetColor(R,G,B);

  dsDrawCylinder(dBodyGetPosition(bodyparts6[4].body),dBodyGetRotation(bodyparts6[4].body), parts6_length, parts6_r);
}


// 位置センサの描画
void drawSensor()
{
 double R,G,B;
 dReal r = 0.04;
 R = 188.0/255.0;
 G = 189.0/255.0;
 B = 194.0/255.0;

 dsSetColor(R,G,B);
 dBodyGetRotation(sensor);
 dsDrawSphere(dBodyGetPosition(sensor), dBodyGetRotation(sensor), r);
}

// 目標位置の描画
void drawP()
{
 dReal tmpP[3];
 dMatrix3 tmpR;

 tmpP[0] = P[0];
 tmpP[1] = P[1];
 tmpP[2] = P[2];

 double R = 255.0/255.0;
 double G = 20.0/255.0;
 double B = 147.0/255.0;

 dsSetColor(R,G,B);

 dRSetIdentity(tmpR);
 dsDrawSphere(tmpP, tmpR, 0.02);
 printf("P= %f %f %f \n",tmpP[0],tmpP[1],tmpP[2]);
}


/*** 制御 ***/
void Pcontrol()
{
  dReal k =  20.0, fMax = 200.0;                   // 比例ゲイン，最大トルク

  for (int j = 1; j < 7; j++) {
    dReal tmp = dJointGetHingeAngle(joint[j]);     // 関節角の取得
    dReal z = THETA[j] - tmp;                      // 残差
    if (z >=   M_PI) z -= 2.0 * M_PI;
    if (z <= - M_PI) z += 2.0 * M_PI;
    dJointSetHingeParam(joint[j],dParamVel, k*z);  // 角速度の設定
    dJointSetHingeParam(joint[j],dParamFMax,fMax); // トルクの設定
  }
}


void  inverseKinematics()
{
  double Px, Py, Pz;
  Px = P[0], Py = P[1], Pz = P[2]; // アーム先端の目標座標P(Px,Py,Pz)
  double a2[3];
  double b2[3];

  double P5x = Px - (l[5] + l[6])*a[0];
  double P5y = Py - (l[5] + l[6])*a[1];
  double P5z = Pz - (l[5] + l[6])*a[2];

  //printf("Target  Position: x=%6.3f y=%6.3f z=%6.3f \n", Px, Py, Pz);

  double tmpL  = sqrt(P5x * P5x + P5y * P5y);
  double P1P   = sqrt(P5x * P5x + P5y * P5y
               + (P5z - (l[0] + l[1])) * (P5z - (l[0] + l[1])));
  double Ca    = (l[2] * l[2] + P1P * P1P -l[3] * l[3])/(2 * l[2] * P1P);  // cosα

  double phi   = atan2(P5z - (l[0] + l[1]), tmpL);                      //φ
  double alpha = atan2(sqrt(1 - Ca * Ca), Ca);                         //α

  double Cb    = (l[2]*l[2] + l[3]*l[3] - P1P*P1P)/(2 * l[2] * l[3]);  //cosβ
  double beta  = atan2(sqrt(1- Cb * Cb), Cb);                          //β


  switch (ANSWER) { // ANSWERはキーボードからの入力で変更
    case 1:
    case 2:
      THETA[1] = atan2(P5y, P5x);
      THETA[2] = M_PI/2 - phi - alpha;
      THETA[3] = M_PI - beta; break;
    case 3:
    case 4:
      THETA[1] = atan2(P5y, P5x);
      THETA[2] = M_PI/2 - phi + alpha;
      THETA[3] = M_PI + beta; break;
    case 5:
    case 6:
      THETA[1] = atan2(P5y, P5x) + M_PI;
      THETA[2] = -(M_PI/2 - phi - alpha);
      THETA[3] = M_PI + beta; break;
    case 7:
    case 8:
      THETA[1] = atan2(P5y, P5x) + M_PI;
      THETA[2] = -(M_PI/2 - phi + alpha);
      THETA[3] = M_PI - beta; break;
  }

  a2[0] = cos(THETA[2]+THETA[3])*(a[0]*cos(THETA[1])+a[1]*sin(THETA[1])) - a[2]*sin(THETA[2]+THETA[3]);
  a2[1] = -a[0]*sin(THETA[1]) + a[1]*cos(THETA[1]);
  a2[2] = sin(THETA[2]+THETA[3])*(a[0]*cos(THETA[1])+a[1]*sin(THETA[1])) + a[2]*cos(THETA[2]+THETA[3]);
  b2[0] = cos(THETA[2]+THETA[3])*(b[0]*cos(THETA[1])+b[1]*sin(THETA[1])) - b[2]*sin(THETA[2]+THETA[3]);
  b2[1] = -b[0]*sin(THETA[1]) + b[1]*cos(THETA[1]);
  b2[2] = sin(THETA[2]+THETA[3])*(b[0]*cos(THETA[1])+b[1]*sin(THETA[1])) + b[2]*cos(THETA[2]+THETA[3]);

  switch (ANSWER) { // ANSWERはキーボードからの入力で変更
    case 1:
    case 3:
    case 5:
    case 7:
      THETA[4] = atan2(a2[1], a2[0]);
      break;
    case 2:
    case 4:
    case 6:
    case 8:
      THETA[4] = atan2(a2[1], a2[0]) + M_PI;
      break;
  }

  THETA[5] = atan2(cos(THETA[4]) * a2[0] + sin(THETA[4]) * a2[1], a2[2]);
  THETA[6] = atan2(sin(THETA[4]) * sin(THETA[5])*b2[0] - cos(THETA[4])*sin(THETA[5])*b2[1], b2[2]);
  //printf("Joint   Angle   : 1=%6.2f 2=%6.2f 3=%6.2f \n",THETA[1],THETA[2],THETA[3]);
  //printf("                  4=%6.2f 5=%6.2f 6=%6.2f \n",THETA[4],THETA[5],THETA[6]);
}


void yugan_a()
{
  // a[0] = sin(T[0])*cos(T[1]);
  // a[1] = sin(T[0])*sin(T[1]);
  // a[2] = -cos(T[0]);
  a[0] = sin(T[0])*cos(T[1]);
  a[1] = sin(T[0])*sin(T[1]);
  a[2] = -cos(T[0]);
}