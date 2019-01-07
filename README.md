# AINT308-Machine-Vision-and-Behavioural-Computing
Repository to hold files for AINT308 module assignments.

The following is an extract from AINT308-OverviewAssessment_r11.pdf by Dr Phil Culverhouse.


## Overview
Students will learn how to (a) actuate servos to control the OWL robot using the Raspberry
Pi , (b) learn how animals use vision to interact with the world, (c) how to use OpenCV to
implement visual routines to make the OWL robot behave like an animal and (d) review
complex vision recognition in current commercial toy robots.

## Assessment
There are two assignments, the first is worth 40% of the module mark, the second 60%.
Both require that you control the servos to provide smooth control with image video
evidence. The reports must be well written with good use of English. To gain first-class
marks you will need to consider animal vision systems in your report, including useful
references to books, academic papers and web resources. Your videos must be short and
demonstrate the required item.
### Assignment 1
Deadlines end February – see DLE for details. Submit report to DLE.
Control servos to do the following:
i. Neck control to pan head in a sinusoidal manner (side to side) where the
head moves fastest in the centre of the range.
ii. Eye control from host computer (key press selection of each)
1. Show stereo control by adjusting PWM to each eye and scan
horizontal axis at a plausible rate, mimicking human eye motion
following a target.
2. Show chameleon like eye motion for 10 seconds.
3. Show two other behaviours that mimic human or animal emotive
eye motion.

### Assignment 2
You will need to use a Smeaton 302/303 laboratory computer and an OWL robot for the
coursework. (indicative effort in brackets)
1. Stereo vision software & eye control

• Using simple Cross-correlation & Servo control

2. Verge onto a slow-moving target with both eyes

3. Give estimate of distance to user, report distance against
measured

4. Track target for 10 seconds over whole field of view of robot

• Using Homography in OpenCV & servo control

5. Calibrate cameras in orthographic mode

6. Show live disparity images

7. Calculate disparity of a target, produce depth map,
demonstrate calibration against ruler, assess errors

• Using Itti & Koch’s bottom-up Saccadic model of stereo eye
control, develop a processing model and apply it to process a scene.
The model should Saccade to salient targets, acquire images, map
distances to targets, on a live stereo video stream.

