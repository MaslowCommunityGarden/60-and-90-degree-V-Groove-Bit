/*This file is part of the Maslow Control Software.
    The Maslow Control Software is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    Maslow Control Software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with the Maslow Control Software.  If not, see <http://www.gnu.org/licenses/>.
    
    Copyright 2014-2017 Bar Smith*/

// This contains all of the Motion commands

#ifndef motion_h
#define motion_h

// These are used for movement tracking and need to be available to the ISR
extern volatile bool movementUpdated;
#if misloopDebug > 0
  extern volatile bool  inMovementLoop;
  extern volatile bool  movementFail;
#endif

void initMotion();
int   coordinatedMove(const float&, const float&, const float&, float);
void  singleAxisMove(Axis*, const float&, const float&);
int   arc(const float&, const float&, const float&, const float&, const float&, const float&, const float&, const float&, const float&, const float&);
float calculateFeedrate(const float&, const float&);
float computeStepSize(const float&);
void movementUpdate();
void motionDetachIfIdle();

#endif
