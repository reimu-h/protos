#ifndef PROTOS_USER_NVME_NVME_REGS_H_
#define PROTOS_USER_NVME_NVME_REGS_H_

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace protos::nvme {

// Controller Register offsets in BAR0 [NVMe Base 2.0 §3.1.3].
constexpr int64_t kRegCap = 0x0000;    // 64-bit Controller Capabilities
constexpr int64_t kRegVs = 0x0008;     // 32-bit Version
constexpr int64_t kRegIntms = 0x000C;  // 32-bit Interrupt Mask Set
constexpr int64_t kRegIntmc = 0x0010;  // 32-bit Interrupt Mask Clear
constexpr int64_t kRegCc = 0x0014;     // 32-bit Controller Configuration
constexpr int64_t kRegCsts = 0x001C;   // 32-bit Controller Status
constexpr int64_t kRegAqa = 0x0024;    // 32-bit Admin Queue Attributes
constexpr int64_t kRegAsq = 0x0028;    // 64-bit Admin SQ Base Address
constexpr int64_t kRegAcq = 0x0030;    // 64-bit Admin CQ Base Address
constexpr int64_t kRegDoorbellsBase = 0x1000;

// CAP bits and field decoders.
constexpr uint64_t kCapCssNvm = 1ULL << 37;

constexpr int CapMqes(const uint64_t cap) {
  return static_cast<int>(cap & 0xFFFFULL) + 1;
}

constexpr int CapToMs(const uint64_t cap) {
  const int units = static_cast<int>((cap >> 24) & 0xFFULL);
  return (units > 0 ? units : 1) * 500;
}

constexpr int CapDstrd(const uint64_t cap) {
  return static_cast<int>((cap >> 32) & 0xFULL);
}

constexpr int CapMpsMin(const uint64_t cap) {
  return static_cast<int>((cap >> 48) & 0xFULL);
}

constexpr int CapMpsMax(const uint64_t cap) {
  return static_cast<int>((cap >> 52) & 0xFULL);
}

// CC (Controller Configuration) bits.
constexpr uint32_t kCcEn = 1u << 0;
constexpr uint32_t kCcCssNvm = 0u << 4;
constexpr uint32_t kCcMps4K = 0u << 7;
constexpr uint32_t kCcAmsRoundRobin = 0u << 11;
constexpr uint32_t kCcShnNormal = 1u << 14;
constexpr uint32_t kCcShnAbrupt = 2u << 14;
constexpr uint32_t kCcShnMask = 3u << 14;
constexpr uint32_t kCcIoSqes64 = 6u << 16;  // 2^6 = 64 bytes
constexpr uint32_t kCcIoCqes16 = 4u << 20;  // 2^4 = 16 bytes

// CSTS (Controller Status) bits.
constexpr uint32_t kCstsRdy = 1u << 0;
constexpr uint32_t kCstsCfs = 1u << 1;
constexpr uint32_t kCstsShstMask = 3u << 2;
constexpr uint32_t kCstsShstNormal = 0u << 2;
constexpr uint32_t kCstsShstOccurring = 1u << 2;
constexpr uint32_t kCstsShstComplete = 2u << 2;

constexpr int64_t SqTailDoorbellOffset(const int qid, const int dstrd) {
  if (qid < 0 || dstrd < 0 || dstrd > 15) {
    return -1;
  }
  return kRegDoorbellsBase + (int64_t{2} * qid) * (int64_t{4} << dstrd);
}

constexpr int64_t CqHeadDoorbellOffset(const int qid, const int dstrd) {
  if (qid < 0 || dstrd < 0 || dstrd > 15) {
    return -1;
  }
  return kRegDoorbellsBase + (int64_t{2} * qid + 1) * (int64_t{4} << dstrd);
}

// Admin command opcodes [NVMe Base 2.0 §5].
constexpr uint8_t kAdminCreateIoSq = 0x01;
constexpr uint8_t kAdminCreateIoCq = 0x05;
constexpr uint8_t kAdminIdentify = 0x06;
constexpr uint8_t kAdminSetFeatures = 0x09;

// Identify CNS (Controller or Namespace Structure) values.
constexpr uint32_t kIdentifyCnsNamespace = 0x00;
constexpr uint32_t kIdentifyCnsController = 0x01;
constexpr uint32_t kIdentifyCnsActiveNsList = 0x02;

// Set Features FID values.
constexpr uint32_t kFeatureNumberOfQueues = 0x07;

// Create I/O SQ/CQ CDW11 flags.
constexpr uint32_t kQueuePhysicallyContiguous = 1u << 0;
constexpr uint32_t kCqInterruptEnabled = 1u << 1;

// NVM Command Set opcodes [NVMe NVM Command Set 1.0 §3.2].
constexpr uint8_t kNvmFlush = 0x00;
constexpr uint8_t kNvmWrite = 0x01;
constexpr uint8_t kNvmRead = 0x02;

// CQE status decoders (given `CompletionEntry::status` bits [15:0] of DW3>>16).
constexpr uint16_t kCqePhaseMask = 1u << 0;
constexpr uint16_t kCqeDnrMask = 1u << 15;

constexpr int CqeStatusType(const uint16_t status) {
  return static_cast<int>((status >> 9) & 0x7u);
}

constexpr int CqeStatusCode(const uint16_t status) {
  return static_cast<int>((status >> 1) & 0xFFu);
}

constexpr bool CqeOk(const uint16_t status) {
  return CqeStatusType(status) == 0 && CqeStatusCode(status) == 0;
}

constexpr bool CqeDoNotRetry(const uint16_t status) {
  return (status & kCqeDnrMask) != 0;
}

// CQE status types and codes referenced for error mapping.
constexpr int kStatusTypeGeneric = 0;
constexpr int kStatusTypeCommandSpecific = 1;
constexpr int kStatusTypeMedia = 2;
constexpr int kStatusCodeInvalidField = 0x02;
constexpr int kStatusCodeLbaOutOfRange = 0x80;
// Command-specific (SCT=1): Attempted Write to Read Only Range (0x82).
constexpr int kStatusCodeWriteToReadOnlyRange = 0x82;

// 64-byte NVMe Submission Queue Entry [NVMe Base 2.0 §4.2].
struct SubmissionEntry {
  uint32_t cdw0 = 0;  // [31:16] CID, [15:8] flags (0=PRP), [7:0] OPC
  uint32_t nsid = 0;
  uint32_t cdw2 = 0;
  uint32_t cdw3 = 0;
  uint64_t mptr = 0;
  uint64_t prp1 = 0;
  uint64_t prp2 = 0;
  uint32_t cdw10 = 0;
  uint32_t cdw11 = 0;
  uint32_t cdw12 = 0;
  uint32_t cdw13 = 0;
  uint32_t cdw14 = 0;
  uint32_t cdw15 = 0;
};
static_assert(sizeof(SubmissionEntry) == 64);
static_assert(alignof(SubmissionEntry) == 8);
static_assert(offsetof(SubmissionEntry, prp1) == 24);
static_assert(offsetof(SubmissionEntry, cdw10) == 40);
static_assert(std::has_unique_object_representations_v<SubmissionEntry>);

// 16-byte NVMe Completion Queue Entry [NVMe Base 2.0 §4.6].
struct CompletionEntry {
  uint32_t dw0 = 0;
  uint32_t dw1 = 0;
  uint16_t sq_head = 0;
  uint16_t sq_id = 0;
  uint16_t cid = 0;
  uint16_t status = 0;  // bit 0 = Phase Tag (P), [15:1] = Status Field
};
static_assert(sizeof(CompletionEntry) == 16);
static_assert(alignof(CompletionEntry) == 4);
static_assert(offsetof(CompletionEntry, sq_head) == 8);
static_assert(offsetof(CompletionEntry, status) == 14);
static_assert(std::has_unique_object_representations_v<CompletionEntry>);

constexpr uint32_t MakeSqCdw0(const uint8_t opcode, const uint16_t cid) {
  return (static_cast<uint32_t>(cid) << 16) | static_cast<uint32_t>(opcode);
}

// Field byte offsets in the 4096-byte Identify Controller and Identify
// Namespace buffers.
constexpr int kIdentifyDataBytes = 4096;
static_assert(kIdentifyDataBytes == 4096);
constexpr int kIdCtrlSerialOffset = 4;
constexpr int kIdCtrlSerialLength = 20;
constexpr int kIdCtrlModelOffset = 24;
constexpr int kIdCtrlModelLength = 40;
constexpr int kIdCtrlMdtsOffset = 77;
constexpr int kIdCtrlNnOffset = 516;
constexpr int kIdCtrlVwcOffset = 525;

constexpr int kIdNsSizeOffset = 0;
constexpr int kIdNsCapOffset = 8;
constexpr int kIdNsNlbafOffset = 25;
constexpr int kIdNsFlbasOffset = 26;
constexpr int kIdNsNmicOffset = 30;
constexpr int kIdNsAttrOffset = 99;
constexpr uint8_t kIdNsAttrWriteProtected = 1u << 0;
constexpr int kIdNsLbaf0Offset = 128;

}  // namespace protos::nvme

#endif  // PROTOS_USER_NVME_NVME_REGS_H_
