# Attitude Estimation

This project implements attitude estimation and propagation of it's covariance using raw gyroscope data. Three different parameterizations of attitude are considered i.e, Euler angles, Direction Cosine Matrix and Quaternions.

Refer [here](https://kvmanohar22.github.io/notes/w02/main.pdf) for mathematical discussion.

This is just a fun project. No real use anywhere. This was done for my own understanding of rotation estimation using the said parameterizations and deriving them from first principles.

The estimates are going to drift along which is clear with the visualization of covariance of estimation in the following videos (covariance is the circles along the ends of each axes).

Data used in this testing has been taken from [here](https://github.com/kalibr/).

## Demos

1. Following video shows estimates when IMU is kept static.

<div class="fig figcenter fighighlight">
  <img src="assets/static.gif">
</div> 

2. IMU is under motion as illustrated in the image stream at the bottom right.

<div class="fig figcenter fighighlight">
  <img src="assets/dynamic.gif">
</div> 


