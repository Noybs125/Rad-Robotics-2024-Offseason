package frc.robot.utils;

public class ExampleObject {
    private double num;

    public ExampleObject(double num){
        this.num = num;
    }
    

    public double getNum(){
        return num;
    }

    public double divNum(double val){
        num /= val;
        return num;
    }

    public double remainderNum(double val){
        num %= val;
        return num;
    }
}
