//METHOD MASTER

//Defines various methods to for calculation
package org.firstinspires.ftc.teamcode;

public class MethodMaster {
    private static double sigmoid(long time,
                                  boolean derivative,
                                  boolean integral,
                                  boolean inverse,
                                  double a, double b, double c){
        //Sigmoid function is NOT log base (*)
        double fzero = Math.log((b/a)/(1-(b/a)));
        double y = a/(1+Math.exp(-c*time-fzero));
        if (integral){
            if (inverse){
                y = (Math.log(Math.exp((c*time)/a)-1)-fzero)/c;
            } else {
                y = a*Math.log(1+Math.exp(c*time+fzero))/c;
            }
        } else if (derivative){
            if (inverse){
                //Not accounted for
            } else {
                y = c*(y*(1-y/a));
            }
        } else {
            if (inverse){
                y = (Math.log(time/(a-time))-fzero)/c;
            }
        }
        return y;
    }
}
