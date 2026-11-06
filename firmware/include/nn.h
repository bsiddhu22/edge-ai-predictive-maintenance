/**
 * @file nn.h
 * @brief int8 inference for a 2-layer dense classifier (24 -> 16 ReLU -> 4).
 *
 * Quantization scheme (same as TensorFlow Lite's int8 spec):
 *   real = scale * (q - zero_point); weights symmetric per tensor (zp = 0);
 *   int32 accumulation; requantization by a Q31 fixed-point multiplier and
 *   shift (gemmlowp SaturatingRoundingDoublingHighMul + RoundingDivideByPOT).
 * The Python reference in ml/quant.py implements the same integer math, so
 * outputs are bit-exact between training host and firmware.
 */
#ifndef PDM_NN_H
#define PDM_NN_H

#include <stdint.h>

#define NN_CLASS_NORMAL       0
#define NN_CLASS_IMBALANCE    1
#define NN_CLASS_MISALIGNMENT 2
#define NN_CLASS_BEARING      3

int32_t nn_requantize(int32_t acc, int32_t multiplier, int32_t shift);
void    nn_quantize_input(const float *features, int8_t *q);
void    nn_infer(const int8_t *q_in, int8_t *logits);
int     nn_argmax(const int8_t *v, int n);
void    nn_softmax(const int8_t *logits, float *probs);
const char *nn_class_name(int c);

/** Multiply-accumulates per inference (for the timing model). */
unsigned nn_macs(void);

#endif /* PDM_NN_H */
