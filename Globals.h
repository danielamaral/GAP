#ifndef GLOBALS_H
#define GLOBALS_H

#include "ProblemData.h"
#include "randomc.h"

class Globals {
public:
    static ProblemData* instance() {
        if (instance_ == 0) {
            instance_ = new ProblemData();
        }
        return instance_;
    }
    static CRandomMersenne* rg() {
        if (rg_ == 0) {
            rg_ = new CRandomMersenne(0);
        }
        return rg_;
    }
	static double Infinity() { return kInfinity_; };

private:
    Globals();
    Globals(const Globals&);
    Globals& operator=(const Globals&);

    static CRandomMersenne* rg_;
    static ProblemData* instance_;
	static double kInfinity_;
};

#endif