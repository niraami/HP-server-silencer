#ifndef __CURVES_H
#define __CURVES_H

typedef struct {
  float in_start;
  float in_end;
  float out_start;
  float out_end;
} CurvePoint;

/**
 * @brief Fan curve structure
 * 
 * Each CurvePoint instance includes two ranges, one called "in" and the other 
 * one "out". If the input PWM is in range of the first two parameters 
 * (in_start & in_end), then the out parameters are used to scale in input PWM 
 * in between them (linear interpolation).
 * Example: curve = {0, 80, 0, 35}; input PWM = 40; output PWM = 17.5
 * 
 */
CurvePoint k_curve[] = {
  { 0, 80, 0, 35 },
  { 80, 90, 35, 60 },
  { 90, 100, 90, 100 }
};

#endif /** __CURVES_H */