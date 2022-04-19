#include "CodeGen.h"
#include <sstream>
#include <iostream>
#include <string>
#include <tuple>
#include <dlfcn.h>
#include <stdexcept>
#include <cppmpc/OrderedSet.h>
#include <cppmpc/SymEngineUtilities.h>
#include <cppmpc/CodeGenerator.h>
#include <symengine/basic.h>
#include <symengine/symbol.h>
#include <symengine/matrix.h>
#include "Constants.h"

using SymEngine::Basic;
using SymEngine::RCP;
using SymEngine::Symbol;

namespace Controllers {

std::string generateParameterizedFunction(
        const SymEngine::DenseMatrix& mat,
        const cppmpc::OrderedSet& parameterOrdering,
        const std::string& functionName) {

    std::stringstream sstr;
    // Get all parameters and variables
    cppmpc::UnorderedSetSymbol parameters = cppmpc::getParameters(mat);

    // Create the representations for the parameters
    cppmpc::MapBasicString parameterRepr;
    for (RCP<const Symbol> parameter : parameters) {
        parameterRepr[parameter] =
                "param[" +
                std::to_string(parameterOrdering.indexOf(parameter)) + "]";
    }

    // Create the representations for the variables. We don't have any, but the
    // codegen code needs it.
    cppmpc::MapBasicString variableRepr;

    //============= Function ===========

    // Function signature
    sstr << "void " << functionName
         << "(const double* param, double* out) {"
         << std::endl;

    // The actual matrix construction code
    sstr << cppmpc::CodeGenerator::generateDenseMatrixCode(
            mat,
            variableRepr,
            parameterRepr,
            "out");

    sstr << "}" << std::endl;

    return sstr.str();
}

ParameterizedMatrixFunction getParameterizedFunction(
        const SymEngine::DenseMatrix& mat,
        const cppmpc::OrderedSet& parameterOrdering) {

    std::string functionName = "matrixFunction";
    std::string functionString = generateParameterizedFunction(
            mat,
            parameterOrdering,
            functionName);
    std::vector<std::string> functionVector {functionString};

    std::string tempFileBase = std::tmpnam(nullptr);
    std::string tempFile = tempFileBase + std::string(".cpp");

    cppmpc::CodeGenerator::writeFunctionsToFile(tempFile, functionVector);

    std::string tempSharedObject = tempFileBase + std::string(".so");

    std::stringstream cmd;
    cmd << CPP_COMPILER_PATH << " -shared " << RUNTIME_COMPILER_FLAGS << " "
        << tempFile << " -o " << tempSharedObject;
    int rt = std::system(cmd.str().c_str());

    if (rt != 0) {
        throw std::runtime_error("Runtime compilation failed");
    }

    // load library
    // TODO(ianruh): I should probably call dlclose at some point
    void* sharedLib = dlopen(tempSharedObject.c_str(), RTLD_LAZY);
    if (!sharedLib) {
        std::stringstream msg;
        msg << "Cannot open library: " << dlerror() << std::endl;
        throw std::runtime_error(msg.str());
    }

    ParameterizedMatrixFunction function =
        (ParameterizedMatrixFunction)dlsym(sharedLib, functionName.c_str());
    
    return function;
}

}
