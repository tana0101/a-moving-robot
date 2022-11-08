#include <bits/stdc++.h>
#include <windows.h>
#include <GL/glut.h>
#define  PI   acos(-1)

#define SIZE  50
#define FLOOR_NUMBER_OF_GRID 10
#define jump_sound_path "jump.wav"
#define walk_sound_path "walk.wav"
#define finger_sound_path "finger.wav"

using namespace std;

// 動作角度參數
int dir = 1;
int finger_dir = 1;
float relbow_joint_angle[4] = { 0, 0, 0, 1 };
float lelbow_joint_angle[4] = { 0, 0, 0, 1 };
float rhand_joint_angle[4] = { 0,0,0,1 };
float lhand_joint_angle[4] = { 0,0,0,1 };
float rfoot_joint_angle = -90, lfoot_joint_angle = -90;
float rfinger_joint_angle = 45, lfinger_joint_angle = -45;
float rknee_joint_angle = 0, lknee_joint_angle = 0;

//視角
float lookat[9] = { 45.0, 70.0, 55.0, 25.0, 0.0, 0.0, 0.0, 1.0, 0.0 };

#define NORMAL_MODE 0   // 預設視角
#define ROBOT_MODE 1    // 跟隨機器人視角
#define WALK_MODE 0     // 走路
#define RUN_MODE 1      // 跑步
#define RWAVE_MODE 0    // 右手揮臂
#define LWAVE_MODE 1    // 左手揮臂
#define WAVE_MODE 2     // 雙手揮臂

// 預設
int mode_orient = NORMAL_MODE; // 視角模式
float r = sqrt(55 * 55 + 20 * 20); // 視角距離
float angle = acos(lookat[0] / r); // 視角角度
float step = 0.5; // 行走速度
float swing = 10; // 揮臂速度
float reduce_swing = 0; // 減小揮臂幅度


/*-----Define a unit box--------*/
/* Vertices of the box */
float  points[][3] = { {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5},
                      {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
                      {-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5},
                      {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5} };
/* face of box, each face composing of 4 vertices */
int    face[][4] = { {0, 3, 2, 1}, {0, 1, 5, 4}, {1, 2, 6, 5},
                    {4, 5, 6, 7}, {2, 3, 7, 6}, {0, 4, 7, 3} };
float  colors[7][3] = { {0.5,0.5,0.5}, {0.7,0.7,0.7}, {0.7,0.5,0.5},
                     {0.5,0.5,0.5}, {0.5,0.7,0.5}, {0.5,0.5,0.7},
                     {1,0.0,0.0} };
/* indices of the box faces */
int    cube[6] = { 0, 1, 2, 3, 4, 5 };

/*-Declare GLU quadric objects, sphere, cylinder, and disk --*/
GLUquadricObj* sphere = NULL, * cylind = NULL, * disk;


/*-Declare car position, orientation--*/
float  self_ang = -90.0, glob_ang = 0.0;
float  position[3] = { 1.0, 5.0, 1.0 };

/*-----Define window size----*/
int width = 512, height = 512;

/*----------------------------------------------------------
 * Procedure to initialize the working environment.
 */
void  myinit()
{
    glClearColor(0.0, 0.0, 0.0, 1.0);      /*set the background color BLACK */
    /*Clear the Depth & Color Buffers */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);
    /*---Create quadratic objects---*/
    if (sphere == NULL) {
        sphere = gluNewQuadric();
        gluQuadricDrawStyle(sphere, GLU_FILL);
        gluQuadricNormals(sphere, GLU_SMOOTH);
    }
    if (cylind == NULL) {
        cylind = gluNewQuadric();
        gluQuadricDrawStyle(cylind, GLU_FILL);
        gluQuadricNormals(cylind, GLU_SMOOTH);
    }
    if (disk == NULL) {
        disk = gluNewQuadric();
        gluQuadricDrawStyle(disk, GLU_FILL);
        gluQuadricNormals(disk, GLU_SMOOTH);
    }

}


/*--------------------------------------------------------
 * Procedure to draw a 1x1x1 cube. The cube is within
 * (-0.5,-0.5,-0.5) ~ (0.5,0.5,0.5)
 */
void draw_cube()
{
    int    i;

    for (i = 0; i < 6; i++) {
        glColor3fv(colors[i]);  /* Set color */
        glBegin(GL_POLYGON);  /* Draw the face */
        glVertex3fv(points[face[i][0]]);
        glVertex3fv(points[face[i][1]]);
        glVertex3fv(points[face[i][2]]);
        glVertex3fv(points[face[i][3]]);
        glEnd();
    }
}

/*---------------------------------------------------------
 * Procedure to draw the floor.
 */
void draw_floor()
{
    int  i, j;

    for (i = 0; i < FLOOR_NUMBER_OF_GRID; i++)
        for (j = 0; j < FLOOR_NUMBER_OF_GRID; j++) {
            if (i == 5) {
                glColor3f(1.0, 0.0, 0.0);
            }
            else if ((i + j) % 2 == 0) glColor3f(1.0, 0.8, 0.8);
            else glColor3f(0.1, 0.1, 0.7);
            glBegin(GL_POLYGON);
            float gridSize = SIZE / FLOOR_NUMBER_OF_GRID;
            glVertex3f((i)*gridSize, 0, (j)*gridSize);
            glVertex3f((i)*gridSize, 0, (j + 1) * gridSize);
            glVertex3f((i + 1) * gridSize, 0, (j + 1) * gridSize);
            glVertex3f((i + 1) * gridSize, 0, (j)*gridSize);
            glEnd();
        }
}

/*-------------------------------------------------------
 * Procedure to draw three axes and the orign
 */
void draw_axes()
{

    /*----Draw a white sphere to represent the original-----*/
    glColor3f(0.9, 0.9, 0.9);

    gluSphere(sphere, 2.0,   /* radius=2.0 */
        12,            /* composing of 12 slices*/
        12);           /* composing of 8 stacks */

    /*----Draw three axes in colors, yellow, meginta, and cyan--*/
    /* Draw Z axis  */
    glColor3f(0.0, 0.95, 0.95);
    gluCylinder(cylind, 0.5, 0.5, /* radius of top and bottom circle */
        10.0,              /* height of the cylinder */
        12,               /* use 12-side polygon approximating circle*/
        3);               /* Divide it into 3 sections */

    /* Draw Y axis */
    glPushMatrix();
    glRotatef(-90.0, 1.0, 0.0, 0.0);  /*Rotate about x by -90', z becomes y */
    glColor3f(0.95, 0.0, 0.95);
    gluCylinder(cylind, 0.5, 0.5, /* radius of top and bottom circle */
        10.0,             /* height of the cylinder */
        12,               /* use 12-side polygon approximating circle*/
        3);               /* Divide it into 3 sections */
    glPopMatrix();

    /* Draw X axis */
    glColor3f(0.95, 0.95, 0.0);
    glPushMatrix();
    glRotatef(90.0, 0.0, 1.0, 0.0);  /*Rotate about y  by 90', x becomes z */
    gluCylinder(cylind, 0.5, 0.5,   /* radius of top and bottom circle */
        10.0,             /* height of the cylinder */
        12,               /* use 12-side polygon approximating circle*/
        3);               /* Divide it into 3 sections */
    glPopMatrix();
    /*-- Restore the original modelview matrix --*/
    glPopMatrix();
}

void draw_blade()
{
    glBegin(GL_POLYGON);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(1.0, 4.0, 0.0);
    glVertex3f(1.0, 8.0, 0.0);
    glVertex3f(-1.0, 8.0, 0.0);
    glVertex3f(-1.0, 4.0, 0.0);
    glEnd();
}

/*-------------------------------------------------------
 * Display callback func. This func. draws three
 * cubes at proper places to simulate a solar system.
 */

void draw_rectangle(double x, double y, double z) {
    glPushMatrix();
    glTranslatef(x / 2, 0, 0);
    glScalef(x, y, z);
    draw_cube();
    glPopMatrix();
}

void draw_finger(float finger_joint_angle) {
    glPushMatrix();
    glRotatef(finger_joint_angle, 0, 1, 0);
    glColor3fv(colors[6]);
    draw_rectangle(0.8, 0.3, 0.3);
    glPopMatrix();
}

void draw_hand(float* hand_joint_angle, float* elbow_joint_angle) {
    glColor3fv(colors[6]);
    glRotatef(-90, 0, 0, 1); // 預設讓手放下

    glPushMatrix();
    glRotatef(hand_joint_angle[0], hand_joint_angle[1], hand_joint_angle[2], hand_joint_angle[3]);
    glutSolidSphere(0.5, 20.0, 20.0); //肩膀
    draw_rectangle(1.5, 0.7, 0.7); //上臂


    glTranslatef(1.5, 0, 0);
    glPushMatrix();
    glColor3fv(colors[6]);
    glRotatef(90, 0, 0, 1); // 預設讓手肘微彎
    glRotatef(elbow_joint_angle[0], elbow_joint_angle[1], elbow_joint_angle[2], elbow_joint_angle[3]);
    glutSolidSphere(0.5, 20.0, 20.0); //手肘

    draw_rectangle(1.5, 0.7, 0.7); //下臂

    glTranslatef(1.5, 0, 0);
    glColor3fv(colors[6]);
    glutSolidSphere(0.3, 20.0, 20.0); //手腕
    // 左手指
    draw_finger(lfinger_joint_angle);
    // 右手指
    draw_finger(rfinger_joint_angle);

    glPopMatrix();
    glPopMatrix();
}

void draw_foot(float foot_joint_angle, float knee_joint_angle) {
    glPushMatrix();
    glRotatef(foot_joint_angle, 0, 0, 1);
    draw_rectangle(1.5, 0.7, 0.7); //大腿
    glTranslatef(1.5, 0, 0);
    glColor3fv(colors[6]);
    glutSolidSphere(0.5, 20.0, 20.0); //膝蓋
    glRotatef(knee_joint_angle, 0, 0, 1);
    draw_rectangle(1.5, 0.7, 0.7); //小腿
    glPopMatrix();
}

void display()
{
    static float  ang_self = 0.0;  /*Define the angle of self-rotate */
    static float  angle = 0.0;

    /*Clear previous frame and the depth buffer */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /*----Define the current eye position and the eye-coordinate system---*/
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(lookat[0], lookat[1], lookat[2], lookat[3], lookat[4], lookat[5], lookat[6], lookat[7], lookat[8]);
    draw_floor();

    //draw_axes();
    /*-------Draw the car body which is a cube----*/
    glTranslatef(position[0], position[1], position[2]);

    glRotatef(self_ang, 0.0, 1.0, 0.0);

    glPushMatrix();              /* Save M1 coord. sys */
    glScalef(2.0, 4.0, 3.0);    /* Scale up the axes */
    draw_cube();
    glPopMatrix();               /* Get M1 back */

    // 頭
    glPushMatrix();
    glTranslatef(0.0, 3.0, 0.0);
    glutSolidSphere(1.0, 20.0, 20.0);
    glPopMatrix();

    // 左手
    glPushMatrix();
    glTranslatef(0.0, 1.5, -1.8);
    draw_hand(lhand_joint_angle, lelbow_joint_angle);
    glPopMatrix();

    // 右手
    glPushMatrix();
    glTranslatef(0.0, 1.5, 1.8);
    draw_hand(rhand_joint_angle, relbow_joint_angle);
    glPopMatrix();

    // 左腳
    glPushMatrix();
    glTranslatef(0.0, -2.0, -0.75);
    draw_foot(lfoot_joint_angle, lknee_joint_angle);
    glPopMatrix();

    // 右腳
    glPushMatrix();
    glTranslatef(0.0, -2.0, 0.75);
    draw_foot(rfoot_joint_angle, rknee_joint_angle);
    glPopMatrix();

    /*-------Swap the back buffer to the front --------*/
    glutSwapBuffers();
    return;
}

/*--------------------------------------------------
 * Reshape callback function which defines the size
 * of the main window when a reshape event occurrs.
 */
void my_reshape(int w, int h)
{

    width = w;
    height = h;

    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (w > h)
        glOrtho(-40.0, 40.0, -40.0 * (float)h / (float)w, 40.0 * (float)h / (float)w,
            0.0, 120);
    else
        glOrtho(-40.0 * (float)w / (float)h, 40.0 * (float)w / (float)h, -40.0, 40.0,
            0.0, 120);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

// sound

void walk_sound() {
    PlaySound(TEXT(walk_sound_path), NULL, SND_ASYNC | SND_FILENAME | SND_NOSTOP);
}

void jump_sound() {
    PlaySound(TEXT(jump_sound_path), NULL, SND_ASYNC | SND_FILENAME);
}

void finger_sound() {
    PlaySound(TEXT(finger_sound_path), NULL, SND_ASYNC | SND_FILENAME | SND_NOSTOP);
}

// motion

void reset() {

    //motion
    dir = 1, finger_dir = 1;

    // hand
    rhand_joint_angle[0] = 0, rhand_joint_angle[1] = 0, rhand_joint_angle[2] = 0, rhand_joint_angle[3] = 1;
    lhand_joint_angle[0] = 0, lhand_joint_angle[1] = 0, lhand_joint_angle[2] = 0, lhand_joint_angle[3] = 1;
    relbow_joint_angle[0] = 0, relbow_joint_angle[1] = 0, relbow_joint_angle[2] = 0, relbow_joint_angle[3] = 1;
    lelbow_joint_angle[0] = 0, lelbow_joint_angle[1] = 0, lelbow_joint_angle[2] = 0, lelbow_joint_angle[3] = 1;
    rfinger_joint_angle = 45, lfinger_joint_angle = -45;

    //foot
    rfoot_joint_angle = -90, lfoot_joint_angle = -90;
    rknee_joint_angle = 0, lknee_joint_angle = 0;

    // y_axis
    position[1] = 5.0;
}

void reset_pos() {
    self_ang = -90.0, glob_ang = 0.0;
    position[0] = 1.0, position[1] = 5.0, position[2] = 1.0;
}

void squat() {
    reset();
    rknee_joint_angle = -90;
    lknee_joint_angle = -90;
    rfoot_joint_angle = 20;
    lfoot_joint_angle = 20;
    position[1] -= 1.5 * sin(20);
    display();
    Sleep(200);
    reset();
}

void jump() {
    jump_sound();
    for (int i = 0; i < 5; i++) {
        position[1] += 0.5;
        display();
        Sleep(20);
    }
    for (int i = 0; i < 5; i++) {
        position[1] -= 0.5;
        display();
        Sleep(20);
    }
}

void wave(int mode) {
    reset();
    if (mode == RWAVE_MODE) {
        for (int i = 0; i < 5; i++) {
            rhand_joint_angle[0] += 18;
            display();
            Sleep(20);
        }
        relbow_joint_angle[1] = 0, relbow_joint_angle[2] = 1, relbow_joint_angle[3] = 0;
        for (int i = 0; i < 5; i++) {
            relbow_joint_angle[0] += 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 10; i++) {
            relbow_joint_angle[0] -= 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 10; i++) {
            relbow_joint_angle[0] += 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 10; i++) {
            relbow_joint_angle[0] -= 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 5; i++) {
            relbow_joint_angle[0] += 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 5; i++) {
            rhand_joint_angle[0] -= 18;
            display();
            Sleep(20);
        }
    }
    else if (mode == LWAVE_MODE) {
        for (int i = 0; i < 5; i++) {
            lhand_joint_angle[0] += 18;
            display();
            Sleep(20);
        }
        lelbow_joint_angle[1] = 0, lelbow_joint_angle[2] = 1, lelbow_joint_angle[3] = 0;
        for (int i = 0; i < 5; i++) {
            lelbow_joint_angle[0] += 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 10; i++) {
            lelbow_joint_angle[0] -= 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 10; i++) {
            lelbow_joint_angle[0] += 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 10; i++) {
            lelbow_joint_angle[0] -= 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 5; i++) {
            lelbow_joint_angle[0] += 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 5; i++) {
            lhand_joint_angle[0] -= 18;
            display();
            Sleep(20);
        }
    }
    else if (mode == WAVE_MODE) {
        for (int i = 0; i < 5; i++) {
            lhand_joint_angle[0] += 18;
            rhand_joint_angle[0] += 18;
            display();
            Sleep(20);
        }
        relbow_joint_angle[1] = 0, relbow_joint_angle[2] = 1, relbow_joint_angle[3] = 0;
        lelbow_joint_angle[1] = 0, lelbow_joint_angle[2] = 1, lelbow_joint_angle[3] = 0;
        for (int i = 0; i < 5; i++) {
            lelbow_joint_angle[0] += 9;
            relbow_joint_angle[0] -= 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 10; i++) {
            lelbow_joint_angle[0] -= 9;
            relbow_joint_angle[0] += 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 10; i++) {
            lelbow_joint_angle[0] += 9;
            relbow_joint_angle[0] -= 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 10; i++) {
            lelbow_joint_angle[0] -= 9;
            relbow_joint_angle[0] += 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 5; i++) {
            lelbow_joint_angle[0] += 9;
            relbow_joint_angle[0] -= 9;
            display();
            Sleep(20);
        }
        for (int i = 0; i < 5; i++) {
            lhand_joint_angle[0] -= 18;
            rhand_joint_angle[0] -= 18;
            display();
            Sleep(20);
        }
    }
}

void dance() {
    rhand_joint_angle[1] = 0, rhand_joint_angle[2] = 1, rhand_joint_angle[3] = 0;
    lhand_joint_angle[1] = 0, lhand_joint_angle[2] = 1, lhand_joint_angle[3] = 0;
    relbow_joint_angle[1] = 0, relbow_joint_angle[2] = 1, relbow_joint_angle[3] = 0;
    lelbow_joint_angle[1] = 0, lelbow_joint_angle[2] = 1, lelbow_joint_angle[3] = 0;
    for (int i = 0; i < 5; i++) {
        rhand_joint_angle[0] -= 18;
        lhand_joint_angle[0] += 18;
        display();
        Sleep(50);
    }
    for (int i = 0; i < 5; i++) {
        relbow_joint_angle[0] += 18;
        lelbow_joint_angle[0] -= 18;
        display();
        Sleep(50);
    }
    for (int i = 0; i < 5; i++) {
        rhand_joint_angle[0] -= 18;
        lhand_joint_angle[0] += 18;
        display();
        Sleep(50);
    }
    for (int i = 0; i < 5; i++) {
        relbow_joint_angle[0] += 18;
        lelbow_joint_angle[0] -= 18;
        display();
        Sleep(50);
    }
    for (int i = 0; i < 5; i++) {
        relbow_joint_angle[0] -= 18;
        lelbow_joint_angle[0] += 18;
        display();
        Sleep(50);
    }
    for (int i = 0; i < 5; i++) {
        rhand_joint_angle[0] += 18;
        lhand_joint_angle[0] -= 18;
        display();
        Sleep(50);
    }
    for (int i = 0; i < 5; i++) {
        relbow_joint_angle[0] -= 18;
        lelbow_joint_angle[0] += 18;
        display();
        Sleep(50);
    }
    for (int i = 0; i < 5; i++) {
        rhand_joint_angle[0] += 18;
        lhand_joint_angle[0] -= 18;
        display();
        Sleep(50);
    }
    reset();
    squat();
    reset();
}

void orient(float ang) {
    lookat[0] = r * cos(ang);
    lookat[2] = r * sin(ang);
}

void look_on_me() { // 讓視角觀看機器人
    lookat[3] = position[0], lookat[4] = position[1], lookat[5] = position[2];
    r = sqrt(pow(lookat[3] - lookat[0], 2) + pow(lookat[5] - lookat[2], 2));
}

void set_moving(int mode_move) { // 設定動作
    if (mode_move == WALK_MODE) {
        step = 0.1; // 行走速度
        swing = 5; // 揮臂速度
        reduce_swing = 30; // 減小揮臂幅度
    }
    else if (mode_move == RUN_MODE) {
        step = 0.5;
        swing = 10;
        reduce_swing = 0;
    }
}

void move(int mode_move) {
    // 設定動作
    set_moving(mode_move);
    // 隱形牆
    float next_step_sin = step * sin(self_ang * PI / 180.0);
    float next_step_cos = step * cos(self_ang * PI / 180.0);
    if (0 < position[0] + next_step_cos && position[0] + next_step_cos < 50 && (position[0] + next_step_cos <= 22.5 || position[0] + next_step_cos >= 32.5))position[0] += next_step_cos;
    if (0 < position[2] - next_step_sin && position[2] - next_step_sin < 50)position[2] -= next_step_sin;
    // 行走揮臂
    if (rfoot_joint_angle > (0 - reduce_swing)) reset(); // 轉換動作時的修正
    if (rfoot_joint_angle == (0 - reduce_swing) || rfoot_joint_angle == (-180 + reduce_swing)) dir *= -1;
    rfoot_joint_angle += (swing * dir);
    lfoot_joint_angle += (swing * -1 * dir);
    rhand_joint_angle[0] += (swing * -1 * dir);
    lhand_joint_angle[0] += (swing * dir);
    // 走路音效
    if (rfoot_joint_angle == -90 || lfoot_joint_angle == -90) {
        walk_sound();
    }
}

void back(int mode_move) {
    // 設定動作
    set_moving(mode_move);
    // 隱形牆
    float next_step_sin = step * sin(self_ang * PI / 180.0);
    float next_step_cos = step * cos(self_ang * PI / 180.0);
    if (0 < position[0] - next_step_cos && position[0] - next_step_cos < 50 && (position[0] - next_step_cos <= 22.5 || position[0] - next_step_cos >= 32.5))position[0] -= next_step_cos;
    if (0 < position[2] + next_step_sin && position[2] + next_step_sin < 50)position[2] += next_step_sin;
    // 行走揮臂
    if (rfoot_joint_angle > (0 - reduce_swing)) reset(); // 轉換動作時的修正
    if (rfoot_joint_angle == (0 - reduce_swing) || rfoot_joint_angle == (-180 + reduce_swing)) dir *= -1;
    rfoot_joint_angle -= (swing * dir);
    lfoot_joint_angle -= (swing * -1 * dir);
    rhand_joint_angle[0] -= (swing * -1 * dir);
    lhand_joint_angle[0] -= (swing * dir);
    // 走路音效
    if (rfoot_joint_angle == -90 || lfoot_joint_angle == -90) {
        walk_sound();
    }
}

/*--------------------------------------------------
 * Keyboard callback func. When a 'q' key is pressed,
 * the program is aborted.
 */
int prev_key; // 上一次按鍵
void my_quit(unsigned char key, int x, int y)
{
    //cout << int(key) << "\n";
    if (int(key) == 27) exit(0); //ESC
    if (key == 'w') {
        if (mode_orient == ROBOT_MODE) {
            look_on_me(); // 讓視角觀看機器人
        }
        move(RUN_MODE);
    }
    else if (int(key) == 87) { // shift + w 走路
        if (mode_orient == ROBOT_MODE) {
            look_on_me(); // 讓視角觀看機器人
        }
        if (prev_key != 87 && prev_key != 83) reset();
        move(WALK_MODE);
    }
    else if (key == 's') {
        if (mode_orient == ROBOT_MODE) {
            look_on_me();
        }
        back(RUN_MODE);
    }
    else if (key == 83) {
        if (mode_orient == ROBOT_MODE) {
            look_on_me();
        }
        if (prev_key != 87 && prev_key != 83) reset();
        back(WALK_MODE);
    }
    else if (key == 'a') {
        self_ang += 10.0;
    }
    else if (key == 'd') self_ang -= 10.0;
    else if (key == 'r') {
        reset(); // 一下r動作復位
        if (prev_key == int('r')) { // 兩下r位置復位
            reset_pos();
        }
    }
    else if (key == 't') {
        if (rfinger_joint_angle == 0 || rfinger_joint_angle == 45) finger_dir *= -1;
        rfinger_joint_angle += (5 * finger_dir);
        lfinger_joint_angle += (5 * -1 * finger_dir);
        // 手指音效
        if (rfinger_joint_angle == 45) {
            finger_sound();
        }
    }
    else if (key == 'c') { // squat
        squat();
    }
    else if (key == 'j') { // jump
        squat();
        jump();
    }
    else if (int(key) == 32) { // space
        jump();
    }
    else if (key == 'q') {
        angle += 0.03f;
        orient(angle);
    }
    else if (key == 'e') {
        angle -= 0.03f;
        orient(angle);
    }
    else if (key == '1') { // 揮左手
        wave(LWAVE_MODE);
    }
    else  if (key == '2') { // 揮右手
        wave(RWAVE_MODE);
    }
    else if (key == '3') {  // 揮雙手
        wave(WAVE_MODE);
    }
    else if (key == '4') { // 跳舞
        dance();
    }
    else if (int(key) == 9) { // tab
        if (mode_orient == ROBOT_MODE) { // 改為預設視角
            r = sqrt(55 * 55 + 20 * 20);
            lookat[0] = 45, lookat[1] = 70, lookat[2] = 55, lookat[3] = 25, lookat[4] = 0, lookat[5] = 0;
            angle = acos(lookat[0] / r); // 計算極座標角度
            mode_orient = NORMAL_MODE;
        }
        else if (mode_orient == NORMAL_MODE) { // 改為跟隨視角
            lookat[0] = position[0] + 50, lookat[1] = 50, lookat[2] = position[2] + 50;
            lookat[3] = position[0], lookat[4] = position[1], lookat[5] = position[2];
            r = sqrt(pow(lookat[3] - lookat[0], 2) + pow(lookat[5] - lookat[2], 2));
            orient(angle);// 修正角度
            mode_orient = ROBOT_MODE;
        }
    }
    display();
    prev_key = int(key);
}

/*---------------------------------------------------
 * Main procedure which defines the graphics environment,
 * the operation style, and the callback func's.
 */
void main(int argc, char** argv)
{
    /*-----Initialize the GLUT environment-------*/
    glutInit(&argc, argv);

    /*-----Depth buffer is used, be careful !!!----*/
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    glutInitWindowSize(width, height);
    glutCreateWindow("robot");

    myinit();      /*---Initialize other state varibales----*/

    /*----Associate callback func's whith events------*/
    glutDisplayFunc(display);
    /*  glutIdleFunc(display); */
    //glutIdleFunc(func);
    glutReshapeFunc(my_reshape);
    glutKeyboardFunc(my_quit);

    glutMainLoop();
}

