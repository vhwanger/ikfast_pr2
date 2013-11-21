#pragma once
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <complex>

namespace ik_right {
    typedef double IKReal;
    class IKSolution
    {
        public:
            void GetSolution(IKReal* psolution, const IKReal* pfree) const;
            const std::vector<int>& GetFree() const { return vfree; }
            struct VARIABLE
            {
                VARIABLE() : fmul(0), foffset(0), freeind(-1), maxsolutions(1) {
                    indices[0] = indices[1] = -1;
                }
                IKReal fmul, foffset; ///< joint value is fmul*sol[freeind]+foffset
                signed char freeind; ///< if >= 0, mimics another joint
                unsigned char maxsolutions; ///< max possible indices, 0 if controlled by free index or a free joint itself
                unsigned char indices[2]; ///< unique index of the solution used to keep track on what part it came from. sometimes a solution can be repeated for different indices. store at least another repeated root
            };

            void GetSolutionIndices(std::vector<unsigned int>& v) const;
            bool Validate() const;
            std::vector<VARIABLE> basesol;       ///< solution and their offsets if joints are mimiced
            std::vector<int> vfree;
    };
    // Computes all IK solutions given a end effector coordinates and the free joints.
    bool ik(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<IKSolution>& vsolutions);

    // Computes the end effector coordinates given the joint values. This function is used to double check ik.
    void fk(const IKReal* joints, IKReal* eetrans, IKReal* eerot);

    // returns the number of free parameters users has to set apriori
    int getNumFreeParameters();

    // the indices of the free parameters indexed by the chain joints
    int* getFreeParameters();

    // the total number of indices of the chain
    int getNumJoints();

    // the size in bytes of the configured number type
    int getIKRealSize();

    // the ikfast version used to generate this file
    const char* getIKFastVersion();

    // the ik type ID
    int getIKType();

    // a hash of all the chain values used for double checking that the correct IK is used.
    const char* getKinematicsHash();
}
