#pragma once

#include "hw/HW5.h"
#include "Utils.h"

class GradientDescentBug : public amp::GDAlgorithm {
    public:
        GradientDescentBug();
        GradientDescentBug(double dStar, double qStar, double attGain, double repGain);
        
        amp::Path2D plan(const amp::Problem2D& problem) override;
        Eigen::Vector2d step(Eigen::Vector2d curPos, const amp::Problem2D& problem);
        void calcWavefront(const amp::Problem2D& problem);
        Eigen::Vector2d wavefrontGradient(Eigen::Vector2d curPos, const amp::Problem2D& problem);

        ~GradientDescentBug() {}
    private:
        double m_qStar, m_dStar, m_attGain, m_repGain, m_wavGain;
        const int maxSteps = 1000;
        const double epsilon = 0.25;
        bool useWavefront = false;
        Eigen::MatrixXi wavefront;
        int xCells = 101;
        int yCells = 101;
};