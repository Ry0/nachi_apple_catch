#include <stdio.h>
#include <stdlib.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#include <unistd.h>
#include <signal.h>

#include "arm_struct.h"


dWorldID      world;                   // 動力学計算用のワールド
dSpaceID      space;                   // 衝突検出用のスペース
dGeomID       ground;                  // 地面のジオメトリID番号
dJointGroupID contactgroup;            // 接触点グループ
dJointGroupID contactgroup2;            // ボール用の接触点グループ
dJointID      joint[7];              // ジョイントのID番号
dJointID      bodyjoint1[8], bodyjoint2[13], bodyjoint3, bodyjoint4[4], bodyjoint5, bodyjoint6[4];

dBodyID       sensor;                  // センサ用のボディID
dJointID      sensor_joint;            // センサ固定用の関節
dsFunctions   fn;                      // ドロースタッフの描画関数

dJointID grasp;

MyObject base, bodyparts1[9], bodyparts2[14], bodyparts3[2], bodyparts4[5], bodyparts5[2], bodyparts6[5];

dReal  THETA[7] = {0.0};             // 関節の目標角度[rad]

dJointID box1joint[5], box2joint[5];
MyObject box1parts[5], box2parts[5];

int  ANSWER = 1;              // 逆運動学の解
int  i,j = 0;

dReal P[3] = {0.25, 0, 0.4};             // 先端の位置
double position[3][3];

// 有顔ベクトル(a,b)
dReal a[3];//?わっからーん
dReal b[3] = {0.0, 0.0, 1.0};//?わっからーん
dReal T[2] = {0.0, M_PI};
dReal l[7] = {0.20, 0.145, 0.33, 0.34, 0.34, 0.073, 0.18+0.04};   // リンクの長さ[m]

static int STEPS = 0;
static int cnt = 0;
/*------------------------------------ボールのやつ--------------------------------------------------*/
typedef struct {       // MyObject構造体
  dBodyID body;        // ボディ(剛体)のID番号（動力学計算用）
  dGeomID geom;        // ジオメトリのID番号(衝突検出計算用）
  double  l,r,m;       // 長さ[m], 半径[m]，質量[kg]
} BallObject;
BallObject ball[3];

static const dReal  BALL_R    = 0.05;
static const dReal  BALL_P[3] = {0.25, 0.25, 0.1};

static void makeBall() {
  dMass m1;

  dReal ball_mass   = 0.1;
  for(int i=0; i<3; i++){
    ball[i].body    = dBodyCreate(world);

    dMassSetZero(&m1);
    dMassSetSphereTotal(&m1,ball_mass,BALL_R);
    dMassAdjust(&m1,ball_mass);
    dBodySetMass(ball[i].body,&m1);

    ball[i].geom = dCreateSphere(space,BALL_R);
    dGeomSetBody(ball[i].geom, ball[i].body);
    dBodySetPosition(ball[i].body,BALL_P[0],BALL_P[1],0.12);
  }
}

static void drawBall()
{
  double R, G, B;

  for(int i=0; i<3; i++){
    R = 1.0 - 0.2*i;
    G = 0;
    B = 0;
    dsSetColor(R,G,B);
    dsDrawSphere(dGeomGetPosition(ball[i].geom),
    dGeomGetRotation(ball[i].geom),BALL_R);

    double *p = (double *) dBodyGetPosition(ball[i].body);
    for(int j=0; j<2; j++){
      position[i][j] = p[j];
    }
    position[i][2] = p[2];
    printf("りんご%dのPosition: x=%6.3f y=%6.3f z=%6.3f\n", i+1, p[0], p[1], p[2]);
  }
  printf("\n");
}


// 簡易 転がり摩擦計算関数
//  [引数]
//    o    : ジオメトリID
//    coef : 転がり摩擦係数
//
// [計算方法]
//    F = f * (W / r)
//       F : 転がり摩擦[N]
//       f : 転がり摩擦係数
//       r : 物体の半径
//
void rolling_function( dGeomID o, dReal coef, dContact *c )
{
  if(o && dGeomGetClass(o) == dSphereClass){
    dBodyID b = dGeomGetBody(o);
    if (!b) return;
    dMass m;
    dBodyGetMass( b, &m );
    dReal* normal = c->geom.normal;      // 垂直抗力ベクトル

    dReal w = m.mass*(normal[2]*9.8);    // 質量, (memo:角度差cosΘ = (normal[0]*0.0 + normal[1]*0.0 + normal[2]*1.0))
    dReal r = dGeomSphereGetRadius( o ); // 半径
    dReal F = coef * (w / r );           // 転がり摩擦(力)
    dReal T = F * r;                     // 転がり摩擦のトルク?

    const dReal* av = dBodyGetAngularVel(b);

    dReal a_speed = sqrt(av[0]*av[0] + av[1]*av[1] + av[2]*av[2]); // 回転スピード
    if(a_speed > 1.0e-5) {
      dReal n_av[3] = { av[0]/a_speed, av[1]/a_speed, av[2]/a_speed }; // 回転方向の正規化
      dBodyAddTorque( b, -n_av[0]*T, -n_av[1]*T, -n_av[2]*T ); // 転がり摩擦をトルクとしてあたえる
    }else{
      dBodySetAngularVel( b, 0.0f, 0.0f, 0.0f ); // 停止
    }
  }
}

/*------------------------------------衝突検出用--------------------------------------------------*/

static void nearCallback( void *data, dGeomID o1, dGeomID o2 )
{
    dBodyID b1 = dGeomGetBody( o1 );
    dBodyID b2 = dGeomGetBody( o2 );
    dContact contact[10];

    // if((o1==ball.geom && o2 == ground)||
    //   (o1==ball.geom && o2 == base.geom)||
    //   (o2==ground && o1 == bodyparts6[3].geom)||
    //   (o1==base.geom && o2==bodyparts6[3].geom)||
    //   (o1==ball.geom && o2==bodyparts6[3].geom)||
    //   (o1==ball.geom && o2==bodyparts6[2].geom)||
    //   (o1==ball.geom && o2==bodyparts6[1].geom)){
    if((o1==ball[0].geom && o2 == bodyparts6[3].geom)||
      (o1==ball[0].geom && o2 == ground)||
      (o1==ball[1].geom && o2 == bodyparts6[3].geom)||
      (o1==ball[1].geom && o2 == ground)||
      (o1==ball[2].geom && o2 == bodyparts6[3].geom)||
      (o1==ball[2].geom && o2 == ground)||
      (o1==ball[0].geom && o2 == ball[1].geom)||
      (o1==ball[1].geom && o2 == ball[2].geom)||
      (o1==ball[2].geom && o2 == ball[0].geom)||
      (o1==ground)||
      //(o2==ground)||s
      (o2 == box1parts[0].geom)||
      (o2 == box1parts[1].geom)||
      (o2 == box1parts[2].geom)||
      (o2 == box1parts[3].geom)||
      (o2 == box1parts[4].geom)||
      (o2 == box2parts[0].geom)||
      (o2 == box2parts[1].geom)||
      (o2 == box2parts[2].geom)||
      (o2 == box2parts[3].geom)||
      (o2 == box2parts[4].geom)){

    if ( b1 && b2 && dAreConnectedExcluding( b1, b2, dJointTypeContact ) )
        return;

    for ( int i=0; i<10; i++ )
    {
        //contact[i].surface.mode = dContactBounce | dContactSoftERP | dContactSoftCFM;
        contact[i].surface.mode = dContactBounce | dContactApprox1;
        contact[i].surface.mu = 50;
        contact[i].surface.bounce = 0.01;
        //contact[i].surface.soft_erp = 0.2;
        //contact[i].surface.soft_cfm = 0.00001;
    }

    int numc = dCollide( o1, o2, 10, &contact[0].geom, sizeof( dContact ) );
    if ( numc > 0 )
    {
        for ( int i=0; i<numc; i++ )
        {
            dJointID c = dJointCreateContact( world, contactgroup, contact+i );
            dJointAttach( c, b1, b2 );

            // 転がり摩擦
            rolling_function( o1, 0.02, contact+i );
            rolling_function( o2, 0.02, contact+i );
        }
    }
  }
}

/*------------------------------------りんごキャッチ!!--------------------------------------------------*/

static void graspBall()
{
  dContact g[10];

  int touch[256];
  if(STEPS==0){
    for(int i=0; i<3; i++){
      touch[i] = dCollide(ball[i].geom, bodyparts6[3].geom, 1, &g[0].geom, sizeof( dContact ));
      if(touch[i]>0){
        double *pos = (double *) dBodyGetPosition(ball[i].body);

        grasp = dJointCreateHinge(world, contactgroup2); // 接触している２つの剛体を接触ジョイントにより拘束
        dJointAttach(grasp, ball[i].body, bodyparts6[3].body);

        dJointSetHingeAxis(grasp,0, 0, 1);
        dJointSetHingeAnchor(grasp, pos[0], pos[1], pos[2]+BALL_R);
        dJointSetHingeParam(grasp,dParamLoStop, 0);
        dJointSetHingeParam(grasp,dParamHiStop, 0);

        printf("ないすかっち\n");
        STEPS++;
        printf("STEPS=%d\n", STEPS);
      }
    }
  }
}



/*** 視点と視線の設定 ***/
void start()
{
  float xyz[3] = {    1.5f, 0.65f, 0.4f};          // 視点[m]
  float hpr[3] = { -160.0f, 4.5f, 0.0f};          // 視線[°]
  dsSetViewpoint(xyz, hpr);                       // 視点と視線の設定
}


void simLoop(int pause)
{

  dSpaceCollide(space,0,*nearCallback);  // 衝突検出関数

  drawBall();
  drawBox1();
  drawBox2();


  yugan_a();
  inverseKinematics();
  printSensorPosition();
  Pcontrol();                                  // P制御
  dWorldStep(world, 0.01);                     // 動力学計算
  drawArmCenter();                                   // ロボットの描画
  drawArmSide();                                   // ロボットの描画
  drawGripper_start();
  drawGripper();
  drawGripper_edge();
  drawBase();


  //drawP();                                     // 目標位置の描画
  drawSensor();                                // 先端位置の描画
  graspBall();


  dJointGroupEmpty(contactgroup); // ジョイントグループを空にする

  cnt++;

}

/*** キー入力関数 ***/
void command2(int cmd)
{
  switch (cmd) {
    case '1':  ANSWER = 1; break;    // 1キーを押すと姿勢１
    case '2':  ANSWER = 2; break;    // 2キーを押すと姿勢２
    case '3':  ANSWER = 3; break;    // 3キーを押すと姿勢３
    case '4':  ANSWER = 4; break;    // 4キーを押すと姿勢４
    case '5':  ANSWER = 5; break;    // 1キーを押すと姿勢１
    case '6':  ANSWER = 6; break;    // 2キーを押すと姿勢２
    case '7':  ANSWER = 7; break;    // 3キーを押すと姿勢３
    case '8':  ANSWER = 8; break;    // 4キーを押すと姿勢４
    case 'j':  P[0] += 0.01; break;   // jキーを押すと先端のx座標が増加
    case 'f':  P[0] -= 0.01; break;   // fキーを押すと先端のx座標が減少
    case 'k':  P[1] += 0.01; break;   // kキーを押すと先端のy座標が増加
    case 'd':  P[1] -= 0.01; break;   // dキーを押すと先端のy座標が減少
    case 'l':  P[2] += 0.01; break;   // lキーを押すと先端のz座標が増加
    case 's':  P[2] -= 0.01; break;   // sキーを押すと先端のz座標が減少
    case 'z':  T[0] += 0.01; break;   // zキーを押すと有顔ベクトルのθが増加
    case 'v':  T[0] -= 0.01; break;   // vキーを押すと有顔ベクトルのθが減少
    case 'x':  T[1] += 0.01; break;   // xキーを押すと有顔ベクトルのφが増加
    case 'c':  T[1] -= 0.01; break;   // cキーを押すと有顔ベクトルのφが減少
    case 'p':  dJointGroupEmpty(contactgroup2); STEPS=0; break;
  }
}


/*** ドロースタッフの設定 ***/
void setDrawStuff()
{
  fn.version = DS_VERSION;                     // バージョン番号
  fn.start   = &start;                         // start関数
  fn.step    = &simLoop;                       // simLoop関数
  fn.command = &command2;                       // command関数
  fn.path_to_textures = "textures";
}

int main(int argc, char **argv)
{
  dInitODE(); //ＯＤＥの初期化
  setDrawStuff();
  world        = dWorldCreate();                  // ワールドの生成
  space        = dHashSpaceCreate(0);             // スペースの生成
  contactgroup = dJointGroupCreate(0);            // 接触グループの生成
  contactgroup2 = dJointGroupCreate(0);            // 接触グループの生成
  ground       = dCreatePlane(space, 0, 0, 1, 0); // 地面の生成
  dWorldSetGravity(world, 0, 0, -9.8);            // 重力の設定
  makeBase();
  makeArm();                                      // アームの生成
  makeSensor();                                   // センサの生成
  makeBall();         // 球の作成
  makeBox1();
  makeBox2();

  dsSimulationLoop(argc, argv, 640, 480, &fn);    // シミュレーションループ
  dSpaceDestroy(space);                           // スペースの破壊
  dWorldDestroy(world);                           // ワールドの破壊
  dCloseODE(); //ＯＤＥの終了
  return 0;
}
