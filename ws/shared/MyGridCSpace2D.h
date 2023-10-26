#pragma once
#include "tools/ConfigurationSpace.h"
#include "hw/HW4.h"

class MyGridCSpace2D : public amp::GridCSpace2D {
    public:
        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max)
            {}
        
        std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;

        /// @brief dtor
        ~MyGridCSpace2D() {}
};

class MyCSpaceConstructor : public amp::GridCSpace2DConstructor {
    public:
        /// @brief Construct a CSpace from a manipulator and an environment
        /// @param manipulator Two link manipulator (consider ussing `ASSERT` to make sure the manipulator is 2D)
        /// @param env Environment
        /// @return Unique pointer to your C-space. 
        /// NOTE: We use a unique pointer here to be able to move the C-space without copying it, since grid discretization
        /// C-spaces can contain a LOT of memory, so copying would be a very expensive operation. Additionally, a pointer is polymorphic
        /// which allows the type to pose as a GridCSpace2D (even though GridCSpace2D is abstract)
        std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;

        ~MyCSpaceConstructor() {}
};
