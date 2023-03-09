package frc.robot;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.DigitalOutput;

public class Bling {

    DigitalOutput m_DIO1 = new DigitalOutput(8);
    DigitalOutput m_DIO2 = new DigitalOutput(9);

    private Timer timer = new Timer();

    int runs = 0;

    public void timerStart(){
        timer = new Timer();
        timer.start();
        m_DIO1.set(false);
        m_DIO2.set(false);
    }

    public void reset(){
        m_DIO1.set(false);
        m_DIO2.set(false);
        timer.reset();
    }

    public void timerReset(){
        timer.reset();
    }

    public void blingRan(){
        runs++;
        if(runs > 200){
            runs = 0;
            m_DIO1.set(false);
            m_DIO2.set(false);
        }
    }

    public void blingSend(int message){
        if(timer.get() > 5){
            if(message == 1){
                m_DIO1.set(true);
                m_DIO2.set(false);
                System.out.println("DIO1 SET HIGH");
                timer.reset();
            }else if(message == 2){
                m_DIO1.set(false);
                m_DIO2.set(true);
                System.out.println("DIO2 SET HIGH");
                timer.reset();
            }
        }
    }
}