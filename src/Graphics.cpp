#include "display/lvgl.h"
#include "generalFunctions.h"
#include "pros/apix.h" //for lvgl graphics
#include "graphics.h"
#include <stdio.h>
#include <errno.h>
#include "Settings.h"

void sayHello() {

bool displayText = true;

if (displayText)  {
  /*Create a Label on the currently active screen*/
    lv_obj_t * label1 =  lv_label_create(lv_scr_act(), NULL);

    /*Modify the Label's text*/
    lv_label_set_text(label1, "7701R - Reseract");

    /* Align the Label to the center
     * NULL means align on parent (which is the screen n ow)
     * 0, 0 at the end means an x, y offset after alignment*/
    lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);



    /*Create an array for the points of the line*/
    static lv_point_t line_points[] = { {5, 5}, {70, 70}, {120, 10}, {180, 60}, {240, 10} };

    /*Create line with default style*/
    lv_obj_t * line1;
    line1 = lv_line_create(lv_scr_act(), NULL);
    lv_line_set_points(line1, line_points, 5);     /*Set the points*/
    lv_obj_align(line1, NULL, LV_ALIGN_IN_TOP_MID, 0, 20);

    /*Create new style (thin light blue)*/
    static lv_style_t style_line2;
    lv_style_copy(&style_line2, &lv_style_plain);
    style_line2.line.color = LV_COLOR_MAKE(0x2e, 0x96, 0xff);
    style_line2.line.width = 2;

    /*Copy the previous line and apply the new style*/
    lv_obj_t * line2 = lv_line_create(lv_scr_act(), line1);
    lv_line_set_style(line2, &style_line2);
    lv_obj_align(line2, line1, LV_ALIGN_OUT_BOTTOM_MID, 0, -20);

    /*Create new style (thick dark blue)*/
    static lv_style_t style_line3;
    lv_style_copy(&style_line3, &lv_style_plain);
    style_line3.line.color = LV_COLOR_MAKE(0x00, 0x3b, 0x75);
    style_line3.line.width = 5;

    /*Copy the previous line and apply the new style*/
    lv_obj_t * line3 = lv_line_create(lv_scr_act(), line1);
    lv_line_set_style(line3, &style_line3);
    lv_obj_align(line3, line2, LV_ALIGN_OUT_BOTTOM_MID, 0, -20);
  }
}



/*Create a normal Text area*/
lv_obj_t * ta1 = lv_ta_create(lv_scr_act(), NULL);

void makeConsole() {
  //make the style
  static lv_style_t TA_Style;

  lv_style_copy(&TA_Style, &lv_style_transp);


  TA_Style.text.color = LV_COLOR_RED;

  /*Create a scroll bar style*/
  static lv_style_t style_sb;
  lv_style_copy(&style_sb, &lv_style_plain);
  style_sb.body.main_color = LV_COLOR_BLACK;
  style_sb.body.grad_color = LV_COLOR_BLACK;
  style_sb.body.border.color = LV_COLOR_WHITE;
  style_sb.body.border.width = 1;
  style_sb.body.border.opa = LV_OPA_70;
  style_sb.body.radius = LV_RADIUS_CIRCLE;
  style_sb.body.opa = LV_OPA_60;


  lv_obj_set_size(ta1, 150, 240);

  lv_ta_set_style(ta1,LV_TA_STYLE_BG,&TA_Style);                     /*Apply the scroll bar style*/
  lv_ta_set_cursor_type(ta1, LV_CURSOR_UNDERLINE);
  lv_ta_set_text(ta1, "Console:\n");    /*Set an initial text*/

}
int numPrints = 0;
void checkConsole() {
  numPrints++;
  if (numPrints > 50) consoleClear();
}

//console functions
void consoleLog(std::string str) {
  const char * data = str.c_str();
  lv_ta_add_text(ta1,data);
  checkConsole();
}
void consoleLog(char data) {
  consoleLog(std::to_string(data));
  checkConsole();
}
void consoleLog(double data) {
  consoleLog(std::to_string(data));
  checkConsole();
}
void consoleLog(float data) {
  consoleLog(std::to_string(data));
  checkConsole();
}
void consoleLog(int data) {
  consoleLog(std::to_string(data));
  checkConsole();
}

void consoleLogN(std::string inStr) {
  std::string str = inStr + "\n";
  const char * data = str.c_str();
  lv_ta_add_text(ta1,data);
  checkConsole();
}
void consoleLogN(char data) {
  consoleLogN(std::to_string(data));
  checkConsole();
}
void consoleLogN(double data) {
  consoleLogN(std::to_string(data));
  checkConsole();
}
void consoleLogN(float data) {
  consoleLogN(std::to_string(data));
  checkConsole();
}
void consoleLogN(int data) {
  consoleLogN(std::to_string(data));
  checkConsole();
}

void consoleClear() {
  lv_ta_set_text(ta1, "Console:\n");
  numPrints = 0;
}

void initializeLVGL() {
  pros::lcd::shutdown();
  makeConsole();
  sayHello();
}

//make the style
static lv_style_t HEEA_Style;

//for rainbow
static unsigned char r=70,g=25,b=240;
static int incR = (rand() % 10 + 3), incG = (rand() % 10 + 3), incB = (rand() % 10 + 3);


void updateStyle() {
  r+= incR;
  g+= incG;
  b+= incB;

  if ((r >= 242) || (r <= 20))  {
    incR = (rand() % 10 + 3) * -incR/fabs(incR);
  }
  if ((g >= 242) || (g <= 20))  {
    incG =  (rand() % 10 + 3) * -incG/fabs(incG);
  }
  if ((b >= 242) || (b <= 20))  {
    incB =  (rand() % 10 + 3) * -incB/fabs(incB);
  }

  HEEA_Style.line.color = LV_COLOR_MAKE(r,g,b);
}

HEEA_Graphics::RotatableShape makeShape() {

  //making shapes
  using namespace HEEA_Graphics;

  int size = 15;

  lv_style_copy(&HEEA_Style, &lv_style_plain);    /*Copy a built-in style to initialize the new style*/
  HEEA_Style.line.width = size/10;
  HEEA_Style.body.main_color = LV_COLOR_BLACK;

/*
  //all the points in a tessaract
  Point(-size,-size,-size,-size), //0
  Point(size,-size,-size,-size), //1
  Point(size,size,-size,-size), //2
  Point(-size,size,-size,-size),//3
  Point(-size,-size,size,-size), //4
  Point(size,-size,size,-size), //5
  Point(size,size,size,-size), //6
  Point(-size,size,size,-size),//7
  Point(-size,-size,-size,size), //8
  Point(size,-size,-size,size), //9
  Point(size,size,-size,size), //10
  Point(-size,size,-size,size),//11
  Point(-size,-size,size,size), //12
  Point(size,-size,size,size), //13
  Point(size,size,size,size), //14
  Point(-size,size,size,size),//15
  */

/*
  std::vector<int> squarePointRenderOrder = {0,1,2,3,0};
  std::vector<Point> squarePoints{Point(-size,-size,0,0),Point(size,-size,0,0),Point(size,size,0,0),Point(-size,size,0,0)};

  std::vector<int> squarePointRenderOrder = {0,1,2,3,0,4,5,6,7,4,7,3,2,6,5,1};
  std::vector<Point> squarePoints{Point(-size,-size,0,0),Point(size,-size,0,0),Point(size,size,0,0),Point(-size,size,0,0)};


  std::vector<int> cubePointRenderOrder = {0,1,2,3,0,4,5,6,7,4,7,3,2,6,5,1};
  std::vector<Point> cubePoints{Point(-size,-size,-size,-size),Point(size,-size,-size,-size), Point(size,size,-size,-size),Point(-size,size,-size,-size), Point(-size,-size,-size,-size),Point(-size,-size,size,-size), Point(size,-size,size,-size), Point(size,size,size,-size)};


  std::vector<int> breadPointRenderOrder = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,1,40,41,42,43,44,5,44,45,46,47,48,9,48,49,50,51,52,14,52,53,54,55,56,19,56,57,58,59,60,24,60,61,62,63,64,29,64,65,66,67,68,34,68,69,70,71,72,39,72,73,74,75,76,37,76,77};
   //std::vector<int> swirlBreadPointRenderOrder = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,20,40,41,42,43,44,22,44,45,46,47,48,24,48,49,50,51,52,26,52,53,54,55,56,28,56,57,58,59,60,30,60,61,62,63,64,32,64,65,66,67,68,34,68,69,70,71,72,36,72,73,74,75,76,38,76,77};
   std::vector<Point> breadPoints = {
     //face 0
      Point(size*4,3*size,-size*14,-size*14),
      Point(size*4,4*size,-size*14,-size*14),
      Point(size*4,5*size,-size*14,-size*14),
      Point(size*4,6*size,-size*14,-size*14),
      Point(size*4,7*size,-size*14,-size*14),
      Point(size*4,8*size,-size*14,-size*14),
      Point(size*4,9*size,-size*14,-size*14),
      Point(size*3,10*size,-size*14,-size*14),
      Point(size*2,11*size,-size*14,-size*14),
      Point(size*2,12*size,-size*14,-size*14),
      Point(size*3,13*size,-size*14,-size*14),
      Point(size*4,14*size,-size*14,-size*14),
      Point(size*5,14*size,-size*14,-size*14),
      Point(size*6,14*size,-size*14,-size*14),
      Point(size*7,14*size,-size*14,-size*14),
      Point(size*8,14*size,-size*14,-size*14),
      Point(size*9,14*size,-size*14,-size*14),
      Point(size*10,14*size,-size*14,-size*14),
      Point(size*11,14*size,-size*14,-size*14),
      Point(size*12,14*size,-size*14,-size*14),
      Point(size*13,13*size,-size*14,-size*14),
      Point(size*14,12*size,-size*14,-size*14),
      Point(size*14,11*size,-size*14,-size*14),
      Point(size*13,10*size,-size*14,-size*14),
      Point(size*12,9*size,-size*14,-size*14),
      Point(size*12,9*size,-size*14,-size*14),
      Point(size*12,8*size,-size*14,-size*14),
      Point(size*12,7*size,-size*14,-size*14),
      Point(size*12,6*size,-size*14,-size*14),
      Point(size*12,5*size,-size*14,-size*14),
      Point(size*12,4*size,-size*14,-size*14),
      Point(size*12,3*size,-size*14,-size*14),
      Point(size*11,3*size,-size*14,-size*14),
      Point(size*10,3*size,-size*14,-size*14),
      Point(size*9,3*size,-size*14,-size*14),
      Point(size*8,3*size,-size*14,-size*14),
      Point(size*7,3*size,-size*14,-size*14),
      Point(size*6,3*size,-size*14,-size*14),
      Point(size*5,3*size,-size*14,-size*14),
      //face 1
      Point(size*4,3*size,size*14,-size*14),
      Point(size*4,4*size,size*14,-size*14),
      Point(size*4,5*size,size*14,-size*14),
      Point(size*4,6*size,size*14,-size*14),
      Point(size*4,7*size,size*14,-size*14),
      Point(size*4,8*size,size*14,-size*14),
      Point(size*4,9*size,size*14,-size*14),
      Point(size*3,10*size,size*14,-size*14),
      Point(size*2,11*size,size*14,-size*14),
      Point(size*2,12*size,size*14,-size*14),
      Point(size*3,13*size,size*14,-size*14),
      Point(size*4,14*size,size*14,-size*14),
      Point(size*5,14*size,size*14,-size*14),
      Point(size*6,14*size,size*14,-size*14),
      Point(size*7,14*size,size*14,-size*14),
      Point(size*8,14*size,size*14,-size*14),
      Point(size*9,14*size,size*14,-size*14),
      Point(size*10,14*size,size*14,-size*14),
      Point(size*11,14*size,size*14,-size*14),
      Point(size*12,14*size,size*14,-size*14),
      Point(size*13,13*size,size*14,-size*14),
      Point(size*14,12*size,size*14,-size*14),
      Point(size*14,11*size,size*14,-size*14),
      Point(size*13,10*size,size*14,-size*14),
      Point(size*12,9*size,size*14,-size*14),
      Point(size*12,9*size,size*14,-size*14),
      Point(size*12,8*size,size*14,-size*14),
      Point(size*12,7*size,size*14,-size*14),
      Point(size*12,6*size,size*14,-size*14),
      Point(size*12,5*size,size*14,-size*14),
      Point(size*12,4*size,size*14,-size*14),
      Point(size*12,3*size,size*14,-size*14),
      Point(size*11,3*size,size*14,-size*14),
      Point(size*10,3*size,size*14,-size*14),
      Point(size*9,3*size,size*14,-size*14),
      Point(size*8,3*size,size*14,-size*14),
      Point(size*7,3*size,size*14,-size*14),
      Point(size*6,3*size,size*14,-size*14),
      Point(size*5,3*size,size*14,-size*14)
   };


   std::vector<int> tessaractPointRenderOrder{0,1,2,3,0,4,5,6,7,4,7,3,2,6,5,1,0,8,9,10,11,8,12,13,14,15,12,15,11,10,14,13,9,1,5,13,14,6,7,15,12,4,0,3,11,10,2};
   std::vector<Point> tessaractPoints{
     Point(-size,-size,-size,-size), //0
     Point(size,-size,-size,-size), //1
     Point(size,size,-size,-size), //2
     Point(-size,size,-size,-size),//3
     Point(-size,-size,size,-size), //4
     Point(size,-size,size,-size), //5
     Point(size,size,size,-size), //6
     Point(-size,size,size,-size),//7
     Point(-size,-size,-size,size), //8
     Point(size,-size,-size,size), //9
     Point(size,size,-size,size), //10
     Point(-size,size,-size,size),//11
     Point(-size,-size,size,size), //12
     Point(size,-size,size,size), //13
     Point(size,size,size,size), //14
     Point(-size,size,size,size)//15
   };
*/
   std::vector<int> teamRPointRenderOrder{0,1,2,3,4,5,6,7,8,9,0,10,11,1,11,12,2,12,13,3,13,14,4,14,15,5,15,16,6,16,17,7,17,18,8,18,19,9,19,10,20,21,22,23,24,25,26,27,28,29,20,30,31,21,31,32,22,32,33,23,33,34,24,34,35,25,35,36,26,36,37,27,37,38,28,38,39,29,39,30,20,21,11,21,22,12,22,23,13,23,24,14,24,25,15,25,26,16,26,27,17,27,28,18,28,29,19,29,20,30,31,1,31,32,2,32,33,3,33,34,4,34,35,5,35,36,6,36,37,7,37,38,8,38,39,9,39,30,0};
   std::vector<Point> teamRPoints{
     Point(-size,-size*3,-size,-size), //0
     Point(-size,size*5,-size,-size), //1
     Point(size*3,size*2,-size,-size), //2
     Point(size,size*0,-size,-size),//3
     Point(size*3.5,-size*3,-size,-size),//4
     Point(size*2.5,-size*3,-size,-size), //5
     Point(size*0,size*0,-size,-size), //6
     Point(size*2,size*2,-size,-size), //7
     Point(-size*0,size*3,-size,-size),//8
     Point(-size*0,-size*3,-size,-size), //9
     //****************************************************************************************************************************************
     Point(-size,-size*3,size,-size), //10
     Point(-size,size*5,size,-size), //11
     Point(size*3,size*2,size,-size), //12
     Point(size,size*0,size,-size),//13
     Point(size*3.5,-size*3,size,-size),//14
     Point(size*2.5,-size*3,size,-size), //15
     Point(size*0,size*0,size,-size), //16
     Point(size*2,size*2,size,-size), //17
     Point(-size*0,size*3,size,-size),//18
     Point(-size*0,-size*3,size,-size), //19
     //************************************************************************************************************************************
     Point(-size,-size*3,-size,size), //20
     Point(-size,size*5,-size,size), //21
     Point(size*3,size*2,-size,size), //22
     Point(size,size*0,-size,size),//23
     Point(size*3.5,-size*3,-size,size),//24
     Point(size*2.5,-size*3,-size,size), //25
     Point(size*0,size*0,-size,size), //26
     Point(size*2,size*2,-size,size), //27
     Point(-size*0,size*3,-size,size),//28
     Point(-size*0,-size*3,-size,size), //29
     //**************************************************************************************************************************************
     Point(-size,-size*3,size,size), //30
     Point(-size,size*5,size,size), //31
     Point(size*3,size*2,size,size), //32
     Point(size,size*0,size,size),//33
     Point(size*3.5,-size*3,size,size),//34
     Point(size*2.5,-size*3,size,size), //35
     Point(size*0,size*0,size,size), //36
     Point(size*2,size*2,size,size), //37
     Point(-size*0,size*3,size,size),//38
     Point(-size*0,-size*3,size,size), //39
   };
/*
std::vector<int> breadPointRenderOrder{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38};
std::vector<Point> breadPoints{
  //face 0
   Point(size*4,3*size,-size*14,-size*14),
   Point(size*4,4*size,-size*14,-size*14),
   Point(size*4,5*size,-size*14,-size*14),
   Point(size*4,6*size,-size*14,-size*14),
   Point(size*4,7*size,-size*14,-size*14),
   Point(size*4,8*size,-size*14,-size*14),
   Point(size*4,9*size,-size*14,-size*14),
   Point(size*3,10*size,-size*14,-size*14),
   Point(size*2,11*size,-size*14,-size*14),
   Point(size*2,12*size,-size*14,-size*14),
   Point(size*3,13*size,-size*14,-size*14),
   Point(size*4,14*size,-size*14,-size*14),
   Point(size*5,14*size,-size*14,-size*14),
   Point(size*6,14*size,-size*14,-size*14),
   Point(size*7,14*size,-size*14,-size*14),
   Point(size*8,14*size,-size*14,-size*14),
   Point(size*9,14*size,-size*14,-size*14),
   Point(size*10,14*size,-size*14,-size*14),
   Point(size*11,14*size,-size*14,-size*14),
   Point(size*12,14*size,-size*14,-size*14),
   Point(size*13,13*size,-size*14,-size*14),
   Point(size*14,12*size,-size*14,-size*14),
   Point(size*14,11*size,-size*14,-size*14),
   Point(size*13,10*size,-size*14,-size*14),
   Point(size*12,9*size,-size*14,-size*14),
   Point(size*12,9*size,-size*14,-size*14),
   Point(size*12,8*size,-size*14,-size*14),
   Point(size*12,7*size,-size*14,-size*14),
   Point(size*12,6*size,-size*14,-size*14),
   Point(size*12,5*size,-size*14,-size*14),
   Point(size*12,4*size,-size*14,-size*14),
   Point(size*12,3*size,-size*14,-size*14),
   Point(size*11,3*size,-size*14,-size*14),
   Point(size*10,3*size,-size*14,-size*14),
   Point(size*9,3*size,-size*14,-size*14),
   Point(size*8,3*size,-size*14,-size*14),
   Point(size*7,3*size,-size*14,-size*14),
   Point(size*6,3*size,-size*14,-size*14),
   Point(size*5,3*size,-size*14,-size*14),
 };
 */
   /*
    std::vector<int> cubePointRenderOrder = {0,1,2,3,0,4,5,6,7,4,7,3,2,6,5,1};
    std::vector<Point> cubePoints = {
      Point(-size,-size,-size,-size), //0
      Point(size,-size,-size,-size), //1
      Point(size,size,-size,-size), //2
      Point(-size,size,-size,-size),//3
      Point(-size,-size,size,-size), //4
      Point(size,-size,size,-size), //5
      Point(size,size,size,-size), //6
      Point(-size,size,size,-size),//7
    };

     //all the points in a tessaract
     Point(-size,-size,-size,-size), //0
     Point(size,-size,-size,-size), //1
     Point(size,size,-size,-size), //2
     Point(-size,size,-size,-size),//3
     Point(-size,-size,size,-size), //4
     Point(size,-size,size,-size), //5
     Point(size,size,size,-size), //6
     Point(-size,size,size,-size),//7
     Point(-size,-size,-size,size), //8
     Point(size,-size,-size,size), //9
     Point(size,size,-size,size), //10
     Point(-size,size,-size,size),//11
     Point(-size,-size,size,size), //12
     Point(size,-size,size,size), //13
     Point(size,size,size,size), //14
     Point(-size,size,size,size),//15
     */

  //create the line that draws the square
  lv_obj_t * line1 = lv_line_create(lv_scr_act(), NULL);

  lv_line_set_style(line1,&HEEA_Style);

  //create the square
  RotatableShape shape(line1,teamRPoints,teamRPointRenderOrder);

  //set attributes of the square
  shape.setPos(100,100,0,0);
  shape.setSpeed(5,2,0,0);

  shape.setAngularVelocity(0,0,0.0,0.2,0.1,0.06);

  return shape;
}
