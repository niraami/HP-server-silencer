#ifndef __CURVES_H
#define __CURVES_H

typedef struct {
  float in_start;
  float in_end;
  float out_start;
  float out_end;
} CurvePoint;

/** Fan curve structure */
CurvePoint k_curve[] = {
  { 0, 80, 0, 35 },
  { 80, 90, 35, 50 },
  { 90, 100, 99, 99 }
};

#endif /** __CURVES_H */