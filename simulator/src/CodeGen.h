#ifndef SRC_CODE_GEN_H_
#define SRC_CODE_GEN_H_
#include <string>
#include <tuple>
#include <cppmpc/OrderedSet.h>
#include <symengine/basic.h>
#include <symengine/symbol.h>
#include <symengine/matrix.h>

namespace Controllers {

// The function takes parameters, and returns the matrix with the values
// replaced. Making sure dimensions match is the problem of the codegen code
// and the caller.
typedef void (*ParameterizedMatrixFunction)(const double* param, double* out);

std::string generateParameterizedFunction(
        const SymEngine::DenseMatrix& mat,
        const cppmpc::OrderedSet& parameterOrdering,
        const std::string& functionName);

// A function, then B function
ParameterizedMatrixFunction getParameterizedFunction(
        const SymEngine::DenseMatrix& mat,
        const cppmpc::OrderedSet& parameterOrdering);

}

#endif // SRC_CODE_GEN_H_
