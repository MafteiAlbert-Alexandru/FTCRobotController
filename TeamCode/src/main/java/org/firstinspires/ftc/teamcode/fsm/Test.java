package org.firstinspires.ftc.teamcode.fsm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.reflect.InvocationTargetException;
import java.util.Random;

@TeleOp
public class Test  {
    public static void main(String[] args) throws IllegalAccessException, InstantiationException, InvocationTargetException, NoSuchMethodException {
        SmartFSM fsm = new SmartFSM();
        fsm.builder()
                .initial(new SmartState() {
            @Override
            void init()
            {
                System.out.println("INIT1");
            }
            @Override
            void update()
            {
                System.out.println("STATE1");
            }
        }).toNext(new SmartTransition() {
                Random random;
            @Override
                    public void init() {
                        random=new Random();
                    }

                    @Override
                    public boolean check() {
                       return random.nextDouble()>0.9;

                    }

                    @Override
                    public void run() {
                        System.out.println("TRANSFORMATION");
                    }
                }).add(new SmartState() {
                    @Override
                    void init() {
                        System.out.println("INIT2");
                    }

                    @Override
                    void update() {
                        System.out.println("STATE2");
                    }
                }).interconnected(new SmartStateTransitionPair[] {
                        new SmartStateTransitionPair() {{
                            state=new SmartState() {
                                @Override
                                void init() {
                                    super.init();
                                }

                                @Override
                                void update() {
                                    super.update();
                                }

                                @Override
                                public int hashCode() {
                                    return super.hashCode();
                                }
                            };
                            transition=new SmartTransition() {
                                @Override
                                void init() {
                                    super.init();
                                }

                                @Override
                                boolean check() {
                                    return super.check();
                                }

                                @Override
                                void run() {
                                    super.run();
                                }
                            };
                        }
                        }
                }).build();
        fsm.build();
        while(true)
        {
            fsm.update();
        }
    }

}
