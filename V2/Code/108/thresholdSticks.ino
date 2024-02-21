int thresholdStick (int pos) {
  
    // get zero centre position
    pos = pos - 1500;  

    // threshold value for control sticks 
    if (pos > 130) {
      pos = pos -130;
    }
    else if (pos < -130) {
      pos = pos +130;
    }
    else {
      pos = 0;
    }

    pos = map(pos, -370,370,-255,255);
    pos = constrain(pos,-255,255);

    return pos;
}



// motion filter to filter motions and compliance

float filter(float prevValue, float currentValue, int filter) {  
  float lengthFiltered =  (prevValue + (currentValue * filter)) / (filter + 1);  
  return lengthFiltered;  
}


