package frc.robot.Sensors;

// Predicts current value from window of history and requested rejection criteria.

/*
https://gregstanleyandassociates.com/whitepapers/FaultDiagnosis/Filtering/Spike-Filter/spike-filter.htm

Overview of spike filtering
The purpose of a spike filter is to suppress extreme changes in measured variable values, since they probably
don’t reflect actual changes in the monitored process.  Small input changes are passed through without
modification.  For example, given existing pump capacity, it may be physically impossible for a drum level to
increase or decrease more than a few percent in one time interval, unless there is a rupture in the drum or
piping.  Or even if it is physically possible, a flow might not normally change more than 10% within one data
sampling interval.  However, in case the extreme change remains in place n times in a row, then the change
must be accepted as real.  For example, there may have been a large change in setpoint, or loss of flow due to
pump failure.  Or even if the change is physically impossible, there may have been an instrument
recalibration or sudden change in sensor bias that persists.

When using a spike filter, it should be placed first in the signal path, ahead of any other filter. That way,
it can be tuned independently of the other filters.  Other filters would dampen the spike to an extent
dependent on their tuning.  In normal conditions, operation of the other filters remains unaffected because
the spike filter passes through routine small variations without modification.

A simple spike filter has two parameters, M and n. M is a maximum change parameter.   M is set so that in
normal operations, the input does not change by as much as M from one time step to the next.  The parameter n
is the number of extreme changes in a row that will be rejected before finally accepting a large change, with
possible values n=0,1,2,…

In this form, the filter stores its previous output and a counter c for the number of times in a row an input
 rate of change has been violated.  The filter would be initialized to c = 0 and the input matching the output. 

An example implementation of a spike filter
FilterFor a "pseudocode" implementation example, we use the filter notation already introduced:

 IF  ( | (x(k) – y(k-1) ) > M |    AND  c < n    )  { 
      // recent big change:  hold previous safe value
       c = c+1
       y(k) = y(k-1)  
}  ELSE  {  
      // normal operation, or else the recent big change must be real after all
      c = 0
      y(k) = x(k)
}

where
x(k) is the raw input at time step k
y(k) is the spike-filtered output at time step k
c counts how many times in a row the input values have been outside the legal range

Since extreme changes may result from failures, or result from sudden setpoint changes made during plant upsets
(and sample intervals may be fairly slow), n will not typically be greater than 1 or 2, or diagnosis could be
significantly delayed.  The n= 0 case is included so that spike filtering may be easily turned off.
 

Copyright 2010 - 2020, Greg Stanley
*/

class SpikeFilter {
    private double Mabsolute; // max absolute difference between 2 values
    private double Mratio; // max absolute ratio of difference between 2 values / previous value
    private int n; // 0 is off; suggest 1 or maybe rarely 2
    private int c;
    private double prevInput;
    private boolean firstTime = true;

    /**
     * 
     * @param Mabsolute difference to suppress
     * @param Mratio difference to suppress
     * @param n count periods, 0 is off; suggest 1 or maybe rarely 2
     */
    public SpikeFilter(double Mabsolute, double Mratio, int n){
        this.Mabsolute = Mabsolute;
        this.Mratio = Mratio;
        this.n = n;
        this.c = 0;
    }

    public double calculate(double rawInput) {
        if(firstTime)
        {
            firstTime = false;
            prevInput = rawInput;
        }

        if((Math.abs(rawInput - prevInput) > Mabsolute ||
            Math.abs((rawInput - prevInput)) > Math.abs(Mratio*prevInput))
           &&
            c < n)
        { // large change; use previous good value
            c++; // count number of times large change
            return prevInput;
        }
        else { // normal input or large change is holding
            c = 0;
            prevInput = rawInput;
            return rawInput;
        }
    }

    public void setLimit(double Mabsolute, double Mratio) {
        this.Mabsolute = Mabsolute;
        this.Mratio = Mratio;
        this.c = 0;
    }
}
