#ifndef LONGVECTORS_H
#define LONGVECTORS_H

#include <array>
#include <cstring>
#include <ostream>
#include <random>
#include <sstream>
#include <string>
#include <type_traits>
#include <variant>

#include <DirectXMath.h>
#include <DirectXPackedVector.h>

#include <Verify.h>

#include "LongVectorTestData.h"
#include "ShaderOpTest.h"
#include "TableParameterHandler.h"
#include "dxc/Support/WinIncludes.h"
#include "dxc/Support/dxcapi.use.h"
#include "dxc/Test/HlslTestUtils.h"

namespace LongVector {

// We don't have std::bit_cast in C++17, so we define our own version.
template <typename ToT, typename FromT>
typename std::enable_if<sizeof(ToT) == sizeof(FromT) &&
                            std::is_trivially_copyable<FromT>::value &&
                            std::is_trivially_copyable<ToT>::value,
                        ToT>::type
bit_cast(const FromT &Src) {
  ToT Dst;
  std::memcpy(&Dst, &Src, sizeof(ToT));
  return Dst;
}

template <typename DataTypeT, typename LongVectorOpTypeT>
class TestConfig; // Forward declaration

class OpTest {
public:
  BEGIN_TEST_CLASS(OpTest)
  END_TEST_CLASS()

  TEST_CLASS_SETUP(ClassSetup)

  BEGIN_TEST_METHOD(BinaryOpTest)
  TEST_METHOD_PROPERTY(L"DataSource",
                       L"Table:LongVectorOpTable.xml#BinaryOpTable")
  END_TEST_METHOD()

  BEGIN_TEST_METHOD(TrigonometricOpTest)
  TEST_METHOD_PROPERTY(L"DataSource",
                       L"Table:LongVectorOpTable.xml#TrigonometricOpTable")
  END_TEST_METHOD()

  BEGIN_TEST_METHOD(UnaryOpTest)
  TEST_METHOD_PROPERTY(L"DataSource",
                       L"Table:LongVectorOpTable.xml#UnaryOpTable")
  END_TEST_METHOD()

  template <typename LongVectorOpTypeT>
  void DispatchTestByDataType(LongVectorOpTypeT OpType, std::wstring DataType,
                              TableParameterHandler &Handler);

  template <typename DataTypeT, typename LongVectorOpTypeT>
  void DispatchTestByVectorSize(LongVectorOpTypeT OpType,
                                TableParameterHandler &Handler);

  template <typename DataTypeT, typename LongVectorOpTypeT>
  void TestBaseMethod(
      LongVector::TestConfig<DataTypeT, LongVectorOpTypeT> &TestConfig,
      size_t VectorSizeToTest);

private:
  dxc::DxcDllSupport m_Support;
  bool m_Initialized = false;
};

using VariantVector =
    std::variant<std::vector<HLSLBool_t>, std::vector<HLSLHalf_t>,
                 std::vector<float>, std::vector<double>, std::vector<int16_t>,
                 std::vector<int32_t>, std::vector<int64_t>,
                 std::vector<uint16_t>, std::vector<uint32_t>,
                 std::vector<uint64_t>>;

template <typename DataTypeT>
void FillShaderBufferFromLongVectorData(std::vector<BYTE> &ShaderBuffer,
                                        std::vector<DataTypeT> &TestData);

template <typename DataTypeT>
void FillLongVectorDataFromShaderBuffer(MappedData &ShaderBuffer,
                                        std::vector<DataTypeT> &TestData,
                                        size_t NumElements);

template <typename DataTypeT> constexpr bool IsFloatingPointType() {
  return std::is_same_v<DataTypeT, float> ||
         std::is_same_v<DataTypeT, double> ||
         std::is_same_v<DataTypeT, HLSLHalf_t>;
}

template <typename DataTypeT> std::string GetHLSLTypeString();

struct LongVectorOpTypeStringToEnumValue {
  std::wstring OpTypeString;
  uint32_t OpTypeValue;
};

template <typename DataTypeT>
DataTypeT GetLongVectorOpType(const LongVectorOpTypeStringToEnumValue *Values,
                              const std::wstring &OpTypeString,
                              std::size_t Length);

enum ValidationType {
  ValidationType_Epsilon,
  ValidationType_Ulp,
};

enum BasicOpType {
  BasicOpType_Binary,
  BasicOpType_Unary,
  BasicOpType_ScalarBinary,
  BasicOpType_EnumValueCount
};

enum BinaryOpType {
  // Below this line to be moved to HLSLOpType (+, -, * etc)
  BinaryOpType_ScalarAdd,
  BinaryOpType_ScalarMultiply,
  BinaryOpType_ScalarSubtract,
  BinaryOpType_ScalarDivide,
  BinaryOpType_ScalarModulus,
  BinaryOpType_Multiply,
  BinaryOpType_Add,
  BinaryOpType_Subtract,
  BinaryOpType_Divide,
  BinaryOpType_Modulus,
  // Below this line to be moved to HLSLMathOpType (min, max etc)
  BinaryOpType_Min,
  BinaryOpType_Max,
  BinaryOpType_ScalarMin,
  BinaryOpType_ScalarMax,
  BinaryOpType_EnumValueCount
};

static const LongVectorOpTypeStringToEnumValue BinaryOpTypeStringToEnumMap[] = {
    {L"BinaryOpType_ScalarAdd", BinaryOpType_ScalarAdd},
    {L"BinaryOpType_ScalarMultiply", BinaryOpType_ScalarMultiply},
    {L"BinaryOpType_ScalarSubtract", BinaryOpType_ScalarSubtract},
    {L"BinaryOpType_ScalarDivide", BinaryOpType_ScalarDivide},
    {L"BinaryOpType_ScalarModulus", BinaryOpType_ScalarModulus},
    {L"BinaryOpType_Add", BinaryOpType_Add},
    {L"BinaryOpType_Multiply", BinaryOpType_Multiply},
    {L"BinaryOpType_Subtract", BinaryOpType_Subtract},
    {L"BinaryOpType_Divide", BinaryOpType_Divide},
    {L"BinaryOpType_Modulus", BinaryOpType_Modulus},
    {L"BinaryOpType_Min", BinaryOpType_Min},
    {L"BinaryOpType_Max", BinaryOpType_Max},
    {L"BinaryOpType_ScalarMin", BinaryOpType_ScalarMin},
    {L"BinaryOpType_ScalarMax", BinaryOpType_ScalarMax},
};

static_assert(_countof(BinaryOpTypeStringToEnumMap) ==
                  BinaryOpType_EnumValueCount,
              "BinaryOpTypeStringToEnumMap size mismatch. Did you "
              "add a new enum value?");

BinaryOpType GetBinaryOpType(const std::wstring &OpTypeString);

enum UnaryOpType {
  UnaryOpType_Clamp,
  UnaryOpType_Initialize,
  UnaryOpType_AsFloat,
  UnaryOpType_AsFloat16,
  UnaryOpType_AsInt,
  UnaryOpType_AsInt16,
  UnaryOpType_AsUint,
  UnaryOpType_AsUint16,
  UnaryOpType_EnumValueCount
};

static const LongVectorOpTypeStringToEnumValue UnaryOpTypeStringToEnumMap[] = {
    {L"UnaryOpType_Clamp", UnaryOpType_Clamp},
    {L"UnaryOpType_Initialize", UnaryOpType_Initialize},
    {L"UnaryOpType_AsFloat", UnaryOpType_AsFloat},
    {L"UnaryOpType_AsFloat16", UnaryOpType_AsFloat16},
    {L"UnaryOpType_AsInt", UnaryOpType_AsInt},
    {L"UnaryOpType_AsInt16", UnaryOpType_AsInt16},
    {L"UnaryOpType_AsUint", UnaryOpType_AsUint},
    {L"UnaryOpType_AsUint16", UnaryOpType_AsUint16},
};

static_assert(_countof(UnaryOpTypeStringToEnumMap) ==
                  UnaryOpType_EnumValueCount,
              "UnaryOpTypeStringToEnumMap size mismatch. Did you add "
              "a new enum value?");

UnaryOpType GetUnaryOpType(const std::wstring &OpTypeString);

enum TrigonometricOpType {
  TrigonometricOpType_Acos,
  TrigonometricOpType_Asin,
  TrigonometricOpType_Atan,
  TrigonometricOpType_Cos,
  TrigonometricOpType_Cosh,
  TrigonometricOpType_Sin,
  TrigonometricOpType_Sinh,
  TrigonometricOpType_Tan,
  TrigonometricOpType_Tanh,
  TrigonometricOpType_EnumValueCount
};

static const LongVectorOpTypeStringToEnumValue
    TrigonometricOpTypeStringToEnumMap[] = {
        {L"TrigonometricOpType_Acos", TrigonometricOpType_Acos},
        {L"TrigonometricOpType_Asin", TrigonometricOpType_Asin},
        {L"TrigonometricOpType_Atan", TrigonometricOpType_Atan},
        {L"TrigonometricOpType_Cos", TrigonometricOpType_Cos},
        {L"TrigonometricOpType_Cosh", TrigonometricOpType_Cosh},
        {L"TrigonometricOpType_Sin", TrigonometricOpType_Sin},
        {L"TrigonometricOpType_Sinh", TrigonometricOpType_Sinh},
        {L"TrigonometricOpType_Tan", TrigonometricOpType_Tan},
        {L"TrigonometricOpType_Tanh", TrigonometricOpType_Tanh},
};

static_assert(_countof(TrigonometricOpTypeStringToEnumMap) ==
                  TrigonometricOpType_EnumValueCount,
              "TrigonometricOpTypeStringToEnumMap size mismatch. Did you add "
              "a new enum value?");

TrigonometricOpType GetTrigonometricOpType(const std::wstring &OpTypeString);

template <typename DataTypeT>
std::vector<DataTypeT> GetInputValueSetByKey(const std::wstring &Key,
                                             bool LogKey = true) {
  if (LogKey)
    WEX::Logging::Log::Comment(
        WEX::Common::String().Format(L"Using Value Set Key: %s", Key.c_str()));
  return std::vector<DataTypeT>(LongVectorTestData<DataTypeT>::Data.at(Key));
}

template <typename DataTypeT>
DataTypeT Mod(const DataTypeT &A, const DataTypeT &B);

template <typename DataTypeInT>
HLSLHalf_t AsFloat16([[maybe_unused]] const DataTypeInT &A) {
  LOG_ERROR_FMT_THROW(L"Programmer Error: Invalid AsFloat16 DataTypeInT: %s",
                      typeid(DataTypeInT).name());
  return HLSLHalf_t();
}

HLSLHalf_t AsFloat16(const HLSLHalf_t &A) { return HLSLHalf_t(A.Val); }

HLSLHalf_t AsFloat16(const int16_t &A) {
  return HLSLHalf_t(LongVector::bit_cast<DirectX::PackedVector::HALF>(A));
}

HLSLHalf_t AsFloat16(const uint16_t &A) {
  return HLSLHalf_t(LongVector::bit_cast<DirectX::PackedVector::HALF>(A));
}

template <typename DataTypeInT>
float AsFloat(const DataTypeInT&) {
  LOG_ERROR_FMT_THROW(L"Programmer Error: Invalid AsFloat DataTypeInT: %s",
                      typeid(DataTypeInT).name());
  return float();
}

float AsFloat(const float &A) { return float(A); }
float AsFloat(const int32_t &A) { return LongVector::bit_cast<float>(A); }
float AsFloat(const uint32_t &A) { return LongVector::bit_cast<float>(A); }

template <typename DataTypeInT>
int32_t AsInt([[maybe_unused]] const DataTypeInT &A) {
  LOG_ERROR_FMT_THROW(L"Programmer Error: Invalid AsInt DataTypeInT: %s",
                      typeid(DataTypeInT).name());
  return int32_t();
}

int32_t AsInt(const float &A) { return LongVector::bit_cast<int32_t>(A); }
int32_t AsInt(const int32_t &A) { return A; }
int32_t AsInt(const uint32_t &A) { return LongVector::bit_cast<int32_t>(A); }

template <typename DataTypeInT>
int16_t AsInt16([[maybe_unused]] const DataTypeInT &A) {
  LOG_ERROR_FMT_THROW(L"Programmer Error: Invalid AsInt16 DataTypeInT: %s",
                      typeid(DataTypeInT).name());
  return int16_t();
}

int16_t AsInt16(const HLSLHalf_t &A) {
  return LongVector::bit_cast<int16_t>(A.Val);
}
int16_t AsInt16(const int16_t &A) { return A; }
int16_t AsInt16(const uint16_t &A) { return LongVector::bit_cast<int16_t>(A); }

template <typename DataTypeInT>
uint16_t AsUint16([[maybe_unused]] const DataTypeInT &A) {
  LOG_ERROR_FMT_THROW(L"Programmer Error: Invalid AsInt16 DataTypeInT: %s",
                      typeid(DataTypeInT).name());
  return uint16_t();
}

uint16_t AsUint16(const HLSLHalf_t &A) {
  return LongVector::bit_cast<uint16_t>(A.Val);
}
uint16_t AsUint16(const uint16_t &A) { return A; }
uint16_t AsUint16(const int16_t &A) {
  return LongVector::bit_cast<uint16_t>(A);
}

template <typename DataTypeInT>
unsigned int AsUint([[maybe_unused]] const DataTypeInT &A) {
  LOG_ERROR_FMT_THROW(L"Programmer Error: Invalid AsInt16 DataTypeInT: %s",
                      typeid(DataTypeInT).name());
  return unsigned int();
}

unsigned int AsUint(const unsigned int &A) { return A; }
unsigned int AsUint(const float &A) {
  return LongVector::bit_cast<unsigned int>(A);
}
unsigned int AsUint(const int &A) {
  return LongVector::bit_cast<unsigned int>(A);
}

template <typename LongVectorOpTypeT> struct TestConfigTraits {
  TestConfigTraits(LongVectorOpTypeT OpType) : OpType(OpType) {}
  // LongVectorOpTypeT* Enum values. We don't use a UINT because
  // we want the type data.
  LongVectorOpTypeT OpType;
};

template <typename DataTypeT>
bool DoValuesMatch(DataTypeT A, DataTypeT B, float Tolerance, ValidationType);
bool DoValuesMatch(HLSLBool_t A, HLSLBool_t B, float, ValidationType);
bool DoValuesMatch(HLSLHalf_t A, HLSLHalf_t B, float Tolerance,
                   ValidationType ValidationType);
bool DoValuesMatch(float A, float B, float Tolerance,
                   ValidationType ValidationType);
bool DoValuesMatch(double A, double B, float Tolerance,
                   ValidationType ValidationType);

template <typename DataTypeT>
bool DoVectorsMatch(const std::vector<DataTypeT> &ActualValues,
                    const std::vector<DataTypeT> &ExpectedValues,
                    float Tolerance, ValidationType ValidationType);
// Binary ops
template <typename DataTypeT, typename LongVectorOpTypeT>
void ComputeExpectedValues(
    const std::vector<DataTypeT> &InputVector1,
    const std::vector<DataTypeT> &InputVector2,
    TestConfig<DataTypeT, LongVectorOpTypeT> &Config);

// Binary scalar ops
template <typename DataTypeT, typename LongVectorOpTypeT>
void ComputeExpectedValues(
    const std::vector<DataTypeT> &InputVector1, const DataTypeT &ScalarInput,
    TestConfig<DataTypeT, LongVectorOpTypeT> &Config);

// Unary ops
template <typename DataTypeT, typename LongVectorOpTypeT>
void ComputeExpectedValues(
    const std::vector<DataTypeT> &InputVector1,
    TestConfig<DataTypeT, LongVectorOpTypeT> &Config);

template <typename DataTypeT>
void LogLongVector(const std::vector<DataTypeT> &Values,
                   const std::wstring &Name);

template <typename DataTypeT>
void LogScalar(const DataTypeT &Value, const std::wstring &Name);

// Used to pass into LongVectorOpTestBase
template <typename DataTypeT, typename LongVectorOpTypeT> class TestConfig {
public:
  TestConfig() = default;

  TestConfig(UnaryOpType OpType);
  TestConfig(BinaryOpType OpType);
  TestConfig(TrigonometricOpType OpType);

  bool IsBinaryOp() const {
    return BasicOpType == LongVector::BasicOpType_Binary ||
           BasicOpType == LongVector::BasicOpType_ScalarBinary;
  }

  bool IsUnaryOp() const {
    return BasicOpType == LongVector::BasicOpType_Unary;
  }

  bool IsScalarOp() const {
    return BasicOpType == LongVector::BasicOpType_ScalarBinary;
  }

  bool IsAsTypeOp() const { return IsAsTypeOp(OpTypeTraits.OpType);}

  bool HasInputArguments() const {
    // TODO: Right now only clamp has input args. Will need to update this
    // later.
    if constexpr (std::is_same_v<LongVectorOpTypeT, LongVector::UnaryOpType>)
      return IsClampOp();
    else
      return false;
  }

  bool HasFunctionDefinition() const;
  std::string GetOPERAND2String() const;

  // Helpers to get the hlsl type as a string for a given C++ type.
  std::string GetHLSLInputTypeString() const;
  std::string GetHLSLOutputTypeString() const;

  DataTypeT ComputeExpectedValue(const DataTypeT &A, const DataTypeT &B,
                                 BinaryOpType OpType) const;
  DataTypeT ComputeExpectedValue(const DataTypeT &A, const DataTypeT &B) const;
  DataTypeT ComputeExpectedValue(const DataTypeT &A,
                                 TrigonometricOpType OpType) const;
  DataTypeT ComputeExpectedValue(const DataTypeT &A, UnaryOpType OpType) const;
  DataTypeT ComputeExpectedValue(const DataTypeT &A) const;
  void ComputeExpectedValuesForAsTypeOp(
      const std::vector<DataTypeT> &InputVector1);

  void SetInputArgsArrayName(const std::wstring &InputArgsArrayName) {
    this->InputArgsArrayName = InputArgsArrayName;
  }

  void SetInputValueSet1(const std::wstring &InputValueSetName) {
    InputValueSetName1 = InputValueSetName;
  }

  void SetInputValueSet2(const std::wstring &InputValueSetName) {
    InputValueSetName2 = InputValueSetName;
  }

  bool IsClampOp() const {
    if constexpr (std::is_same_v<LongVectorOpTypeT, LongVector::UnaryOpType>)
      return OpTypeTraits.OpType == LongVector::UnaryOpType_Clamp;
    else
      return false;
  }

  std::vector<DataTypeT> GetInputValueSet1() const {
    return GetInputValueSet(1);
  }

  std::vector<DataTypeT> GetInputValueSet2() const {
    return GetInputValueSet(2);
  }

  std::vector<DataTypeT> GetInputArgsArray() const;

  float GetTolerance() const { return Tolerance; }
  LongVector::ValidationType GetValidationType() const {
    return ValidationType;
  }

  std::string GetCompilerOptionsString(size_t VectorSize) const;

  LongVector::VariantVector& GetExpectedVector() {
    return ExpectedVector;
  }

  bool VerifyOutput(MappedData &ShaderOutData, size_t NumElements);

private:
  bool IsAsTypeOp(LongVector::UnaryOpType OpType) const;
  bool IsAsTypeOp(LongVector::BinaryOpType) const { return false;}
  bool IsAsTypeOp(LongVector::TrigonometricOpType) const { return false; }

  std::string HLSLOutputTypeString(LongVector::UnaryOpType OpType) const;

  bool
  ResolveOutputTypeAndVerifyOutput(MappedData &ShaderOutData, size_t NumElements);

  // Templated version to be used when the output data type does not match the
  // input data type.
  template <typename OutputDataTypeT>
  bool VerifyOutput(MappedData &ShaderOutData, size_t NumElements);

  std::vector<DataTypeT> GetInputValueSet(size_t ValueSetIndex) const;

  // To be used for the value of -DOPERATOR
  std::string OperatorString;
  // To be used for the value of -DFUNC
  std::string IntrinsicString;
  LongVector::BasicOpType BasicOpType = LongVector::BasicOpType_EnumValueCount;
  float Tolerance = 0.0;
  LongVector::ValidationType ValidationType =
      LongVector::ValidationType::ValidationType_Epsilon;
  LongVector::TestConfigTraits<LongVectorOpTypeT> OpTypeTraits;
  std::wstring InputValueSetName1 = L"DefaultInputValueSet1";
  std::wstring InputValueSetName2 = L"DefaultInputValueSet2";
  // No default args array
  std::wstring InputArgsArrayName = L"";
  // Default the TypedOutputVector to use DataTypeT, Ops that don't have a
  // matching output type will override this.
  LongVector::VariantVector ExpectedVector = std::vector<DataTypeT>{};
}; // class LongVector::TestConfig

}; // namespace LongVector

#include "LongVectors.tpp"

#endif // LONGVECTORS_H