#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>


#include "../include/autonomous.h"
#include "motor-control.h"


// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }


void movetwoinch(){
 wait(10000, msec);
 driveTo(6.5,3000);
}
void exampleAuton() {
 // Use this for tuning linear and turn pid
 driveTo(60, 3000);


 turnToAngle(90, 2000);
 turnToAngle(135, 2000);
 turnToAngle(150, 2000);
 turnToAngle(160, 2000);
 turnToAngle(165, 2000);
 turnToAngle(0, 2000);
 driveTo(-60, 3000);
}


void exampleAuton2() {
 moveToPoint(24, 24, 1, 2000, false);
 moveToPoint(48, 48, 1, 2000, true);
 moveToPoint(24, 24, -1, 2000, true);
 moveToPoint(0, 0, 1, 2000, true);
 correct_angle = 0;
 driveTo(24, 2000, false, 8);
 turnToAngle(90, 800, false);
 turnToAngle(180, 800, true);
}


void rightsidelow(){
 moveToPoint(1, -40, -1, 2500, false, 5);//drive to score low goal
 intake2.spin(fwd,5,volt);
 wait(750, msec);
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,12,volt);
 turnToPoint(-5, -36, 1, 1000);//turn to face balls
 moveToPoint(-30, -15, 1, 2000, false, 8);//drive to goal area
 turnToPoint(-30, 0, 1, 1000);//turn to face goal
 moveToPoint(-34, -37, -1, 600, false, 6);//go back and score
 intake2.spin(fwd,12,volt);
 wait(1750, msec);
 scraper.set(true); //scraper down
 intake2.spin(fwd,0,volt);
 resetChassis();
 moveToPoint(-25, 20, 1, 2250, false, 4);//drive to matchloader
 moveToPoint(-35, -37, -1, 350, false, 4);//go back and score
 intake2.spin(fwd,12,volt);
 wait(1500, msec);


 
  //moveToPoint(6.5, 25, 1, 2500, false, 5);//drive to balls
}


void rightsidepush(){
 intake1.spin(fwd,12,volt);
 turnToPoint(10, 33.5, 1, 260); //first stack, increased x coord to 9.5, 250 to 275 to 260, x coord to 10.5 to 10
 driveTo(12,2100,false,5); //distance before matchloader is put down, 13.7 to 14, then to 14.5, then to 15, then to 14, 2000 to 2100, 4 to 3.7, 13.7 to 12
 scraper.set(true); //scraper down
 wait(500,msec); // 400 to 500
 driveTo(2,500,false,7);
 moveToPoint(43.5,1,1,1500,false,10); //drive to matchload
 turnToPoint(44.2,-10,1,500);
 intake1.spin(reverse, 12, volt);
 wait(300, msec);
 intake1.spin(fwd, 12, volt);
 wait(200,msec); //added
 driveTo(17,1700,true,9); //drive into matchload, 14.6 changed to 14.8, then 14.7, changed to 14.5, then to 15, max voltage decreased, then to 14
 //driveChassis(30, 30); //aaron shen magic
 wait(200,msec); //wait from 270 to 235
intake1.spin(reverse, 12, volt);
intake1.spin(fwd,12,volt);
 wait(150,msec); //added
 moveToPoint(45,10.5,-1,1100,true,10); //back out of matchload, was 45.2 but changed to 45, then changed to 44, then 44.5, then 45 and 1100, then 43 to move more left from 45
 driveTo(-10, 500,true,12); // -7 to -10
 intake1.spin(reverse,12,volt);
 wait(300,msec);
 intake1.spin(fwd,12,volt);
 intake2.spin(fwd,12,volt); //score into long goal
 wait(2000,msec); // increased from 2000 to 2050 ms
 scraper.set(false);
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,0,volt);
 driveTo(15, 500, true, 12); //
 turnToAngle(-85, 400, true, 12); //increased speed
 driveTo(14.7, 850, true, 12); //13.5 to 13.9, time out increased by 50 ms from 800 to 850, DECREASED DISTANCE FROM 17 TO 16
 turnToAngle(0, 300, true, 12); //350 to 400
 driveTo(36,1300, true,8);// changed to 33 then 36, 1300
 turnToAngle(-90,9000,true,3); //-90 to -85, then to -95, then to -70, then to -10
 wait(500,msec);
}


void rightside4push(){
 intake1.spin(fwd,12,volt);
 turnToPoint(9, 33.5, 1, 4000); //first stack
 driveTo(8.6,2000,false,7);
 scraper.set(true); //scraper down
 wait(400,msec);
 driveTo(2,500,false,4);
 moveToPoint(43.7,2,1,9000,false,9); //drive to matchload
 turnToPoint(43.7,-10,1,1000);
 driveTo(-7, 1250, false, 11);
 intake2.spin(fwd, 12, volt);
 wait(1600, msec);
 scraper.set(false);
 driveTo(8, 1500, true, 12);
 turnToAngle(-90, 1000, true, 12);
 driveTo(3.5, 1000, true, 12);
 turnToAngle(0, 1000, true, 12);
 driveTo(19,9000, false,5);
}


void leftandmid(){
 intake1.spin(fwd,12,volt);
 moveToPoint(-14,31.5,1,1400,false,6); //first stack, 31.5 to 29
 scraper.set(true);
 wait(100,msec);
 driveTo(-4.5,500,false,10); //come back to allign with midgoal
 //boomerang(-10,10,-1,-135,0.1,1500,false,8); //align with mid goal, y from 20to 10, -10 x to -14
 turnToAngle(-100, 500,false,6);
 driveChassis(-3.4,-3.4); //drive to mid goal
 intake2.spin(reverse, 12, volt);
 intake1.spin(reverse,12,volt);
 wait(420,msec); // before we get to the midgoal, outtake the top stage, when you getg to the midgoal outtake both for one second and then score
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,0,volt);
 wait(300,msec);
 midgoal.set(true);
 intake1.spin(fwd, 12, volt);//score
 wait(1500,msec);
 intake1.spin(fwd,0,volt);
 driveChassis(0,0);
 midgoal.set(false);
 moveToPoint(-18,0,1,1300,false,9); //drive to matchload, -20 to -10
 turnToAngle(181,500,true,5);
 scraper.set(true);
 intake1.spin(fwd,12,volt);
 wait(100, msec);
 driveTo(5,1250,false,6); //drive into matchload
 intake1.spin(fwd,12,volt);
 wait(1150,msec);
 moveToPoint(-18.4,8,-1,800,false,9); //align with long goal
 //turnToAngle(173,9000,false,4);
 driveTo(-12,800,false,11); //drive into long goal
 scraper.set(false);
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,0,volt);
 wait(700, msec);
 intake2.spin(fwd, 12, volt);
 intake1.spin(fwd, 12 ,volt);
 wait(1500,msec);
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,0,volt);
 driveTo(15, 500, true, 12);//descore
 turnToAngle(-90, 350, true, 12);
 driveTo(-10.5, 500, true, 12);
 turnToAngle(-180, 350, true, 12);
 driveTo(-32,1250, true,11);
 turnToAngle(-90,9000,true,6);
}


void leftside7(){
 intake1.spin(fwd,12,volt);
 moveToPoint(-11.5,31,1,9000,false,4); //first stack
 turnToAngle(-160,9000,false,4);
 moveToPoint(-21,-5,1,9000,false,6); //align with long goal
 turnToAngle(180,9000,false,4);
 driveTo(-16,2000,false,10); //drive into long goal
 intake2.spin(fwd,12,volt);
 scraper.set(true);
 wait(1150,msec);
 intake2.spin(fwd,0,volt);
 moveToPoint(-20,0,1,9000,false,8); //align with matchloader
 turnToAngle(185,9000,false,8);
 wait(100,msec);
 driveChassis(6,6);
 wait(1000, msec);
 driveChassis(0,0); //drive into matchloader
 moveToPoint(-22,6,-1,9000,false,6); //align with long goal
 turnToAngle(18,9000,false,4);
 driveTo(-19,1500,false,6); //drive into long goal
 intake2.spin(fwd,12,volt);
}

void autonskills(){
 intake1.spin(fwd,12,volt);
 turnToPoint(13.5, 33.5, 1, 260); //first stack, increased x coord to 9.5, 250 to 275 to 260, x coord to 10.5 to 10
 driveTo(3.5,2000,false,2.3); //distance before matchloader is put down, 13.7 to 14, then to 14.5, then to 15, then to 14, 2000 to 2100, 4 to 3.7, 13.7 to 12
 wait(200, msec);
 scraper.set(true); //scraper down
 driveTo(4, 1000, false, 4);
 wait(1000,msec); // 400 to 500
 driveTo(2,500,false,4);
 moveToPoint(42,1,1,1500,false,9); //drive to long goal area


 intake1.spin(reverse,12,volt);
 intake2.spin(reverse,12,volt);
 wait(200,msec);
 turnToPoint(40,-10,1, 1500); // drive into long goal
 driveTo(-18,1000,false,4);
 intake1.spin(fwd,12,volt);
 intake2.spin(fwd,12,volt);
 wait(2500,msec);
 


 driveTo(16,1700,true,5); //drive into matchload, 14.6 changed to 14.8, then 14.7, changed to 14.5, then to 15, max voltage decreased, then to 14
 wait(50, msec);
 moveToPoint(45, -13, 1, 1500, false, 6);
 wait(50, msec);
 driveTo(10, 1700, true, 5);
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,12,volt);
 wait(2500,msec); //wait from 270 to 235
 intake1.spin(reverse, 12, volt);
 intake1.spin(fwd,12,volt);
 wait(100,msec); //added
 moveToPoint(45,11,-1,1100,true,10); //back out of matchload, was 45.2 but changed to 45, then changed to 44, then 44.5, then 45 and 1100, then 43 to move more left from 45
 driveTo(-10, 500,true,12); // -7 to -10
 intake1.spin(reverse,12,volt);
 intake2.spin(reverse,12,volt);
 wait(300,msec);
 intake1.spin(fwd,12,volt);
 intake2.spin(fwd,12,volt); //score into long goal
 wait(3500,msec); // increased from 2000 to 2050 ms
 scraper.set(false);
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,0,volt);
 driveTo(15, 500, true, 12); //
 turnToAngle(-85, 400, true, 12); //increased speed
 driveTo(13.94, 850, true, 12); //13.5 to 13.9, time out increased by 50 ms from 800 to 850, DECREASED DISTANCE FROM 17 TO 16
 turnToAngle(-1, 300, true, 12); //350 to 400
 driveTo(30, 2300, true,4);// changed to 33 then 36, 1300 // push
 driveTo(-24, 1300, true,8); // drive out of long goal
 turnToAngle(-85, 400, true, 12); //turn to other long goal
  
 intake1.spin(fwd,12,volt); // second stack
 driveTo(43, 1300, true, 8);
 turnToAngle(-65, 400, true, 12);
 driveTo(43, 1000, true, 2.5);
 scraper.set(true);
 driveTo(13, 1000, true, 2.5);
 wait(800, msec);

 moveToPoint(-55,-5,1,2000,false,9); //drive to matchload area, -20 to -10
 turnToAngle(181,500,true,4);
 driveTo(-10,500,false,4); //drive to long goal
 intake1.spin(fwd,12,volt);
 intake2.spin(fwd,12,volt);
 wait(2500,msec);

 driveTo(15, 2000, true, 5); // drive to matchload
 wait(50, msec);
 moveToPoint(-53, -10, 1, 2500, true, 7);
 wait(50, msec);
 driveTo(8, 2000, true, 5);
 intake2.spin(fwd,0,volt);
 intake1.spin(fwd,12,volt);
 wait(2500,msec); //wait from 270 to 235
 intake1.spin(reverse, 12, volt);
 intake1.spin(fwd,12,volt);
 wait(100,msec); //added

 driveTo(-8, 2000, true, 8); // drive back to long goal and score it
 wait(50, msec);
 moveToPoint(-53, -10, 1, 2500, true, 7);
 wait(50, msec);
 driveTo(-24, 2000, true, 8);
 intake1.spin(reverse, 12, volt);
 intake2.spin(reverse, 12, volt);
 wait(100, msec);                            
 intake1.spin(fwd, 12, volt);
 intake2.spin(fwd, 12, volt);
 wait(4000, msec);

 scraper.set(false);

 driveTo(15, 500, true, 12); //
 turnToAngle(-85, 400, true, 12); //increased speed
 driveTo(14.1, 850, true, 12); //13.5 to 13.9, time out increased by 50 ms from 800 to 850, DECREASED DISTANCE FROM 17 TO 16
 turnToAngle(-1, 300, true, 12); //350 to 400
 driveTo(30, 2300, true,5);// changed to 33 then 36, 1300 // push
 driveTo(-24, 1300, true,8); // drive out of long goal
 turnToAngle(85, 400, true, 12); //turn to other long goal

 moveToPoint(-55, -33, 1, 25000, true, 7);
 scraper.set(true);
 driveTo(16,1000,true,5);

}

void autonskillsActual(){ // get balls align to loader matchload go to other side 
  /*driveTo(32, 1000, true, 8);
  turnToAngle(-90, 400, true, 7);
  scraper.set(true);
  wait(50,msec);
  intake1.spin(fwd,12,volt);
  driveTo(15, 1000, true, 7); // into the matchloader
  wait(1500,msec);
  driveTo(-10, 500, true, 6); 
  wait(50, msec);
  scraper.set(false);
  descore.set(true);
  intake1.spin(fwd, 0, volt);
  turnToAngle(48, 1000, true, 8);
  wait(60, msec);
  driveTo(24.5, 2000, true, 11);
  wait(50, msec); 
  turnToAngle(90, 1000, true, 6);
  driveTo(70, 4000, true, 11); // drive in the corridor
  boomerang(96, 33, 1, 180, 0.2, 1000, true, 7); // allign with long goal
  turnToAngle(90, 300, false, 8);
  
  driveTo(-16, 500, true, 9); // drive into long goal
  wait(50, msec);*/
  /*
  scraper.set(true);
  intake1.spin(reverse, 12, volt); 
  intake2.spin(reverse, 12, volt); 
  wait(300, msec);
  intake1.spin(fwd, 12, volt); 
  intake2.spin(fwd, 12, volt); 
  wait(2000, msec);
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 0, volt);
  driveTo(32.3, 1200, true, 8); // drive into matchload
  wait(2000, msec);
  intake1.spin(fwd, 0, volt);
  driveTo(-30.5, 1000, true, 8); // drive into long goal
  intake1.spin(reverse, 12, volt); 
  intake2.spin(reverse, 12, volt); 
  wait(400, msec);
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 12, volt);
  wait(3000, msec);

  scraper.set(false);

  intake1.spin(fwd, 0, volt);
  intake2.spin(fwd, 0, volt);
  */

 /* driveTo(11.3, 2000, false, 9); //come out of long goal
  turnToAngle(90, 400, false, 10);
  wait(50, msec);
  driveTo(93, 2500, true, 10);
  turnToAngle(60, 500, false, 8); // 30 to 45
  driveTo(27, 1000, true, 9);
  //moveToPoint(98, -60, 1, 5000, true, 9);
  turnToAngle(0, 300, false, 7);
  intake1.spin(fwd, 12, volt);
  intake2.spin(fwd, 0, volt);
  scraper.set(true);
  driveTo(19, 1000, true, 9);  // drive into matchload
  wait(3000, msec);
  */

  driveTo(-2,1000,false,8);
  wait(100,msec);
  scraper.set(false);
  turnToAngle(180,1400,false,5);
  driveTo(0.01,100,false,4);
  wait(500,msec);
  turnToAngle(90,1000,false,9);
  driveTo(1,1000,true,9);
  descore.set(true);
  intake1.spin(fwd, 0, volt);
  intake2.spin(fwd, 0, volt);
  driveTo(70,4000,true,11);



  
  
  
  
 
}




/*
void leftside7(){
 intake1.spin(fwd,12,volt); //intake on when auton starts
 moveToPoint(-11,11,1,9000,false,11); //Moves to right before the balls
 scraper.set(true); //puts down scraper
 moveToPoint(-11.5,31,1,9000,false,4); //first stack
 scraper.set(false); //puts up scraper after intaking first set of balls
 turnToAngle(-160,9000,false,6); //idk
 moveToPoint(-21,-5,1,9000,false,8); //align with mid goal - idk
 turnToAngle(180,9000,false,6); //idk
 driveTo(-16,2000,false,6); //drive into mid goal
 intake2.spin(fwd,12,volt); //scoring into mid goal
 scraper.set(true); //putting down scraper for matchloader
 wait(1150,msec); //waiting for some reason
 intake2.spin(fwd,0,volt); //stopping intake
 moveToPoint(-35,0,1,9000,false,6); //align with matchloader (right in front of it)
 turnToPoint(-35,-100,1,9000); //turning towards matchloader
 wait(100,msec); //waiting for some reason
 intake1.spin(fwd,12,volt); //starting intake for matchloader
 driveTo(18,3000,false,6); //drive into matchloader
 wait(100,msec); //waiting for some reason
 //driveTo(-2,500,false, 6);
 //driveTo(2,500,false,6);
 moveToPoint(-35,11,-1,9000,false,8); //moving into long goal
 intake2.spin(fwd,12,volt); //scoring into long goal
 moveToPoint(-35,0,1,9000,false,10); //moving back from matchloader
 intake2.spin(fwd,0,volt); //stopping scoring intake
 intake1.spin(fwd,12,volt); //starting storing intake
 moveToPoint(35,11,1,10000,false,10); //moving to right before second set of balls
 scraper.set(true); //putting down scraper over the balls;
 moveToPoint(35,31,1,9000,false,10);
 moveToPoint(59,0,1,9000,false,10); //moving behind long goal
 moveToPoint(59,11,-1,9000,false,8); //moving inside long goal
 intake2.spin(fwd,12,volt); //scoring into long goal
}
*/


/*
void leftside(){
 descore.set(true); //descore up
 intake1.spin(fwd,12,volt); //start intake
 intake2.spin(fwd,12,volt);
 intake3.spin(reverse,12,volt);
 wait(20, msec);
 driveTo(22.5, 2500, false, 5);//drive to balls
 scraper.set(true); //scraper down
 driveTo(8, 1250, true, 5); //drive more get 3rd ball, msec);
 driveTo(-10, 1500, true); //back up
 wait(200, msec);
 scraper.set(false); //scraper up
 turnToAngle(80, 1500, true); //turn to goal
 intake1.spin(fwd,0,volt); //stop intake
 intake2.spin(fwd,0,volt);
 intake3.spin(fwd,0,volt);
 driveTo(-35, 1000, true); //back away from low goal
 turnToAngle(210, 1500, true); //turn to face goal
 driveTo(-26, 1500, false, 5); //drive to goal
 wait(500, msec);
 descore.set(false); //descore down
 wait(250, msec);
 intake1.spin(fwd,12,volt); //start intake
 intake2.spin(fwd,12,volt);
 intake3.spin(reverse,12,volt);
 wait(1250, msec);
 scraper.set(true); //scraper down
 descore.set(true); //descore up
 driveTo(25, 1000, true, 7.5); //drive to getballs
 turnToAngle(-155,1000,false);
 driveTo(25, 1000, true, 7.5);
 wait(1000, msec);
 turnToAngle(-150,1000,false);
 driveTo(-40, 1000, true,8); //back to goal
 descore.set(false); //descore down
 wait(250, msec);                   
}
*/







