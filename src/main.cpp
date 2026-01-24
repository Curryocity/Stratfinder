#include <iostream>
#include "zEngine.hpp"
#include "zSolver.hpp"
#include "util.hpp"
#include "player.hpp"

void init(){
    util::init();
    zEngine::init();
}

int main() {

    // We don't have GUI yet ;(

    init();
    zSolver s;

 
    bool backwallq = false;
    int max_t = 50;
    double threshold = 1e-9;
    std::string content;
    
    
    if(false){
        s.toggleLog(false);
        for(int speed = 83; speed <= 255; speed ++){
            for(int slow = 0; slow <= 6; slow ++){
                s.setEffect(speed, slow);
                for(double t_mm = 2; t_mm <= 12; t_mm += 1){
                    for(double x = 0.125; x <= 100; x += 0.0625){
                        zSolver::fullStrat strat = s.optimalSolver(x, t_mm);
                        bool hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::normal, strat);
                        if(hasJump) std::cout << content;
                        hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::ladder, strat);
                        if(hasJump) std::cout << content;
                        hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::blockage, strat);
                        if(hasJump) std::cout << content;
                    }
                }
            }
        }
        s.toggleLog(true);
    }

    
   

    if(true){
        s.setEffect(55, 3);
        bool hasJump = s.poss(29.8125, 3, max_t, 1e-5, backwallq, content, zSolver::normal);
        s.printLog();
        std::cout << content;
    }
    
    if(false){
        int speed = 8, slow = 6;
        player p(speed,slow);
        p.setF(0);

        p.sj(1,0,1);
        std::cout << "Speed: " << speed << ", Slow: " << slow << ",  Vz: " << util::df(p.Vz()) << "\n";

        zEngine e(speed, slow);

        e.setVz(-0.0590420844924972);
        e.s45(1);
        e.sj45(11);

        std::cout << "Speed: " << speed << ", Slow: " << slow << ",  Vz: " << util::df(e.Vz()) << "\n";

        
    }



    return 0;
}

