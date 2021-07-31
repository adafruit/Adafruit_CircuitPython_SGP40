# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_sgp40`
================================================================================

CircuitPython library for the Adafruit SGP40 Air Quality Sensor / VOC Index Sensor Breakouts


* Author(s): Bryan Siepert
             Keith Murray

Implementation Notes
--------------------

**Hardware:**

* Adafruit SGP40 Air Quality Sensor Breakout - VOC Index <https://www.adafruit.com/product/4829>
* In order to use the `measure_raw` function, a temperature and humidity sensor which
  updates at at least 1Hz is needed (BME280, BME688, SHT31-D, SHT40, etc. For more, see:
  https://www.adafruit.com/category/66)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

 * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

"""
from time import sleep
from struct import unpack_from
import adafruit_bus_device.i2c_device as i2c_device

VOCALGORITHM_SAMPLING_INTERVAL = (1.)
VOCALGORITHM_INITIAL_BLACKOUT = (45.)
VOCALGORITHM_VOC_INDEX_GAIN = (230.)
VOCALGORITHM_SRAW_STD_INITIAL = (50.)
VOCALGORITHM_SRAW_STD_BONUS = (220.)
VOCALGORITHM_TAU_MEAN_VARIANCE_HOURS = (12.)
VOCALGORITHM_TAU_INITIAL_MEAN = (20.)
VOCALGORITHM_INITI_DURATION_MEAN = (3600. * 0.75)
VOCALGORITHM_INITI_TRANSITION_MEAN = (0.01)
VOCALGORITHM_TAU_INITIAL_VARIANCE = (2500.)
VOCALGORITHM_INITI_DURATION_VARIANCE = ((3600. * 1.45))
VOCALGORITHM_INITI_TRANSITION_VARIANCE = (0.01)
VOCALGORITHM_GATING_THRESHOLD = (340.)
VOCALGORITHM_GATING_THRESHOLD_INITIAL = (510.)
VOCALGORITHM_GATING_THRESHOLD_TRANSITION = (0.09)
VOCALGORITHM_GATING_MAX_DURATION_MINUTES = ((60. * 3.))
VOCALGORITHM_GATING_MAX_RATIO = (0.3)
VOCALGORITHM_SIGMOID_L = (500.)
VOCALGORITHM_SIGMOID_K = (-0.0065)
VOCALGORITHM_SIGMOID_X0 = (213.)
VOCALGORITHM_VOC_INDEX_OFFSET_DEFAULT = (100.)
VOCALGORITHM_LP_TAU_FAST = (20.0)
VOCALGORITHM_LP_TAU_SLOW = (500.0)
VOCALGORITHM_LP_ALPHA = (-0.2)
VOCALGORITHM_PERSISTENCE_UPTIME_GAMMA = ((3. * 3600.))
VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING = (64.)
VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR__FIX16_MAX = (32767.)
FIX16_MAXIMUM = 0x7FFFFFFF
FIX16_MINIMUM = 0x80000000
FIX16_OVERFLOW = 0x80000000
FIX16_ONE = 0x00010000


class DFRobot_vocalgorithmParams:
    def __init__(self):
        self.mvoc_index_offset = 0
        self.mtau_mean_variance_hours = 0
        self.mgating_max_duration_minutes = 0
        self.msraw_std_initial = 0
        self.muptime = 0
        self.msraw = 0
        self.mvoc_index = 0
        self.m_mean_variance_estimator_gating_max_duration_minutes = 0
        self.m_mean_variance_estimator_initialized = 0
        self.m_mean_variance_estimator_mean = 0
        self.m_mean_variance_estimator_sraw_offset = 0
        self.m_mean_variance_estimator_std = 0
        self.m_mean_variance_estimator_gamma = 0
        self.m_mean_variance_estimator_gamma_initial_mean = 0
        self.m_mean_variance_estimator_gamma_initial_variance = 0
        self.m_mean_variance_estimator_gamma_mean = 0
        self.m_mean_variance_estimator__gamma_variance = 0
        self.m_mean_variance_estimator_uptime_gamma = 0
        self.m_mean_variance_estimator_uptime_gating = 0
        self.m_mean_variance_estimator_gating_duration_minutes = 0
        self.m_mean_variance_estimator_sigmoid_l = 0
        self.m_mean_variance_estimator_sigmoid_k = 0
        self.m_mean_variance_estimator_sigmoid_x0 = 0
        self.m_mox_model_sraw_mean = 0
        self.m_sigmoid_scaled_offset = 0
        self.m_adaptive_lowpass_a1 = 0
        self.m_adaptive_lowpass_a2 = 0
        self.m_adaptive_lowpass_initialized = 0
        self.m_adaptive_lowpass_x1 = 0
        self.m_adaptive_lowpass_x2 = 0
        self.m_adaptive_lowpass_x3 = 0


class DFRobot_VOCAlgorithm:

    def __init__(self):
        self.params = DFRobot_vocalgorithmParams()

    def _f16(self, x):
        if x >= 0:
            return int((x)*65536.0 + 0.5)
        else:
            return int((x)*65536.0 - 0.5)

    def _fix16_from_int(self, a):
        return int(a * FIX16_ONE)

    def _fix16_cast_to_int(self, a):
        return int(a) >> 16

    def _fix16_mul(self, inarg0, inarg1):
        inarg0 = int(inarg0)
        inarg1 = int(inarg1)
        A = (inarg0 >> 16)
        if inarg0 < 0:
            B = (inarg0 & 0xFFFFFFFF) & 0xFFFF
        else:
            B = inarg0 & 0xFFFF
        C = (inarg1 >> 16)
        if inarg1 < 0:
            D = (inarg1 & 0xFFFFFFFF) & 0xFFFF
        else:
            D = inarg1 & 0xFFFF
        AC = (A * C)
        AD_CB = (A * D + C * B)
        BD = (B * D)
        product_hi = (AC + (AD_CB >> 16))
        ad_cb_temp = ((AD_CB) << 16) & 0xFFFFFFFF
        product_lo = ((BD + ad_cb_temp)) & 0xFFFFFFFF
        if product_lo < BD:
            product_hi = product_hi+1
        if ((product_hi >> 31) != (product_hi >> 15)):
            return FIX16_OVERFLOW
        product_lo_tmp = product_lo & 0xFFFFFFFF
        product_lo = (product_lo - 0x8000) & 0xFFFFFFFF
        product_lo = (
            product_lo-((product_hi & 0xFFFFFFFF) >> 31)) & 0xFFFFFFFF
        if product_lo > product_lo_tmp:
            product_hi = product_hi-1
        result = (product_hi << 16) | (product_lo >> 16)
        result += 1
        return result

    def _fix16_div(self, a, b):
        a = int(a)
        b = int(b)
        if b == 0:
            return FIX16_MINIMUM
        if a >= 0:
            remainder = a
        else:
            remainder = (a*(-1)) & 0xFFFFFFFF
        if b >= 0:
            divider = b
        else:
            divider = (b*(-1)) & 0xFFFFFFFF
        quotient = 0
        bit = 0x10000
        while (divider < remainder):
            divider = divider << 1
            bit <<= 1
        if not bit:
            return FIX16_OVERFLOW
        if (divider & 0x80000000):
            if (remainder >= divider):
                quotient |= bit
                remainder -= divider
            divider >>= 1
            bit >>= 1
        while bit and remainder:
            if (remainder >= divider):
                quotient |= bit
                remainder -= divider
            remainder <<= 1
            bit >>= 1
        if (remainder >= divider):
            quotient += 1
        result = quotient
        if ((a ^ b) & 0x80000000):
            if (result == FIX16_MINIMUM):
                return FIX16_OVERFLOW
            result = -result
        return result

    def _fix16_sqrt(self, x):
        x = int(x)
        num = x & 0xFFFFFFFF
        result = 0
        bit = 1 << 30
        while (bit > num):
            bit >>= 2
        for n in range(0, 2):
            while (bit):
                if (num >= result + bit):
                    num = num-(result + bit) & 0xFFFFFFFF
                    result = (result >> 1) + bit
                else:
                    result = (result >> 1)
                bit >>= 2
            if n == 0:
                if num > 65535:
                    num = (num - result) & 0xFFFFFFFF
                    num = ((num << 16) - 0x8000) & 0xFFFFFFFF
                    result = ((result << 16) + 0x8000) & 0xFFFFFFFF
                else:
                    num = ((num << 16) & 0xFFFFFFFF)
                    result = ((result << 16) & 0xFFFFFFFF)
                bit = 1 << 14
        if (num > result):
            result += 1
        return result

    def _fix16_exp(self, x):
        x = int(x)
        exp_pos_values = [self._f16(2.7182818), self._f16(
            1.1331485), self._f16(1.0157477), self._f16(1.0019550)]
        exp_neg_values = [self._f16(0.3678794), self._f16(
            0.8824969), self._f16(0.9844964), self._f16(0.9980488)]
        if (x >= self._f16(10.3972)):
            return FIX16_MAXIMUM
        if (x <= self._f16(-11.7835)):
            return 0
        if (x < 0):
            x = -x
            exp_values = exp_neg_values
        else:
            exp_values = exp_pos_values
        res = FIX16_ONE
        arg = FIX16_ONE
        for i in range(0, 4):
            while (x >= arg):
                res = self._fix16_mul(res, exp_values[i])
                x -= arg
            arg >>= 3
        return res

    def vocalgorithm_init(self):
        self.params.mvoc_index_offset = (
            self._f16(VOCALGORITHM_VOC_INDEX_OFFSET_DEFAULT))
        self.params.mtau_mean_variance_hours = self._f16(
            VOCALGORITHM_TAU_MEAN_VARIANCE_HOURS)
        self.params.mgating_max_duration_minutes = self._f16(
            VOCALGORITHM_GATING_MAX_DURATION_MINUTES)
        self.params.msraw_std_initial = self._f16(
            VOCALGORITHM_SRAW_STD_INITIAL)
        self.params.muptime = self._f16(0.)
        self.params.msraw = self._f16(0.)
        self.params.mvoc_index = 0
        self._vocalgorithm__init_instances()

    def _vocalgorithm__init_instances(self):
        self._vocalgorithm__mean_variance_estimator__init()
        self._vocalgorithm__mean_variance_estimator__set_parameters(self._f16(
            VOCALGORITHM_SRAW_STD_INITIAL), self.params.mtau_mean_variance_hours, self.params.mgating_max_duration_minutes)
        self._vocalgorithm__mox_model__init()
        self._vocalgorithm__mox_model__set_parameters(self._vocalgorithm__mean_variance_estimator__get_std(
        ), self._vocalgorithm__mean_variance_estimator__get_mean())
        self._vocalgorithm__sigmoid_scaled__init()
        self._vocalgorithm__sigmoid_scaled__set_parameters(
            self.params.mvoc_index_offset)
        self._vocalgorithm__adaptive_lowpass__init()
        self._vocalgorithm__adaptive_lowpass__set_parameters()

    def _vocalgorithm_get_states(self, state0, state1):
        state0 = self._vocalgorithm__mean_variance_estimator__get_mean()
        state1 = _vocalgorithm__mean_variance_estimator__get_std()
        return state0, state1

    def _vocalgorithm_set_states(self, state0, state1):
        self._vocalgorithm__mean_variance_estimator__set_states(
            params, state0, state1, self._f16(VOCALGORITHM_PERSISTENCE_UPTIME_GAMMA))
        self.params.msraw = state0

    def _vocalgorithm_set_tuning_parameters(self, voc_index_offset, learning_time_hours, gating_max_duration_minutes, std_initial):
        self.params.mvoc_index_offset = self._fix16_from_int(voc_index_offset)
        self.params.mtau_mean_variance_hours = self._fix16_from_int(
            learning_time_hours)
        self.params.mgating_max_duration_minutes = self._fix16_from_int(
            gating_max_duration_minutes)
        self.params.msraw_std_initial = self._fix16_from_int(std_initial)
        self._vocalgorithm__init_instances()

    def vocalgorithm_process(self, sraw):
        if ((self.params.muptime <= self._f16(VOCALGORITHM_INITIAL_BLACKOUT))):
            self.params.muptime = self.params.muptime + \
                self._f16(VOCALGORITHM_SAMPLING_INTERVAL)
        else:
            if (((sraw > 0) and (sraw < 65000))):
                if ((sraw < 20001)):
                    sraw = 20001
                elif((sraw > 52767)):
                    sraw = 52767
                self.params.msraw = self._fix16_from_int((sraw - 20000))
            self.params.mvoc_index = self._vocalgorithm__mox_model__process(
                self.params.msraw)
            self.params.mvoc_index = self._vocalgorithm__sigmoid_scaled__process(
                self.params.mvoc_index)
            self.params.mvoc_index = self._vocalgorithm__adaptive_lowpass__process(
                self.params.mvoc_index)
            if ((self.params.mvoc_index < self._f16(0.5))):
                self.params.mvoc_index = self._f16(0.5)
            if self.params.msraw > self._f16(0.):
                self._vocalgorithm__mean_variance_estimator__process(
                    self.params.msraw, self.params.mvoc_index)
                self._vocalgorithm__mox_model__set_parameters(self._vocalgorithm__mean_variance_estimator__get_std(
                ), self._vocalgorithm__mean_variance_estimator__get_mean())
        voc_index = self._fix16_cast_to_int(
            (self.params.mvoc_index + self._f16(0.5)))
        return voc_index

    def _vocalgorithm__mean_variance_estimator__init(self):
        self._vocalgorithm__mean_variance_estimator__set_parameters(
            self._f16(0.), self._f16(0.), self._f16(0.))
        self._vocalgorithm__mean_variance_estimator___init_instances()

    def _vocalgorithm__mean_variance_estimator___init_instances(self):
        self._vocalgorithm__mean_variance_estimator___sigmoid__init()

    def _vocalgorithm__mean_variance_estimator__set_parameters(self, std_initial, tau_mean_variance_hours, gating_max_duration_minutes):
        self.params.m_mean_variance_estimator_gating_max_duration_minutes = gating_max_duration_minutes
        self.params.m_mean_variance_estimator_initialized = 0
        self.params.m_mean_variance_estimator_mean = self._f16(0.)
        self.params.m_mean_variance_estimator_sraw_offset = self._f16(0.)
        self.params.m_mean_variance_estimator_std = std_initial
        self.params.m_mean_variance_estimator_gamma = self._fix16_div(self._f16((VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING * (VOCALGORITHM_SAMPLING_INTERVAL / 3600.))
                                                                                ), (tau_mean_variance_hours + self._f16((VOCALGORITHM_SAMPLING_INTERVAL / 3600.))))
        self.params.m_mean_variance_estimator_gamma_initial_mean = self._f16(((VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING * VOCALGORITHM_SAMPLING_INTERVAL)
                                                                              / (VOCALGORITHM_TAU_INITIAL_MEAN + VOCALGORITHM_SAMPLING_INTERVAL)))
        self.params.m_mean_variance_estimator_gamma_initial_variance = self._f16(((VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING * VOCALGORITHM_SAMPLING_INTERVAL)
                                                                                  / (VOCALGORITHM_TAU_INITIAL_VARIANCE + VOCALGORITHM_SAMPLING_INTERVAL)))
        self.params.m_mean_variance_estimator_gamma_mean = self._f16(0.)
        self.params.m_mean_variance_estimator__gamma_variance = self._f16(0.)
        self.params.m_mean_variance_estimator_uptime_gamma = self._f16(0.)
        self.params.m_mean_variance_estimator_uptime_gating = self._f16(0.)
        self.params.m_mean_variance_estimator_gating_duration_minutes = self._f16(
            0.)

    def _vocalgorithm__mean_variance_estimator__set_states(self, mean, std, uptime_gamma):
        self.params.m_mean_variance_estimator_mean = mean
        self.params.m_mean_variance_estimator_std = std
        self.params.m_mean_variance_estimator_uptime_gamma = uptime_gamma
        self.params.m_mean_variance_estimator_initialized = true

    def _vocalgorithm__mean_variance_estimator__get_std(self):
        return self.params.m_mean_variance_estimator_std

    def _vocalgorithm__mean_variance_estimator__get_mean(self):
        return (self.params.m_mean_variance_estimator_mean + self.params.m_mean_variance_estimator_sraw_offset)

    def _vocalgorithm__mean_variance_estimator___calculate_gamma(self, voc_index_from_prior):
        uptime_limit = self._f16(
            (VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR__FIX16_MAX - VOCALGORITHM_SAMPLING_INTERVAL))
        if self.params.m_mean_variance_estimator_uptime_gamma < uptime_limit:
            self.params.m_mean_variance_estimator_uptime_gamma = (
                self.params.m_mean_variance_estimator_uptime_gamma + self._f16(VOCALGORITHM_SAMPLING_INTERVAL))

        if self.params.m_mean_variance_estimator_uptime_gating < uptime_limit:
            self.params.m_mean_variance_estimator_uptime_gating = (
                self.params.m_mean_variance_estimator_uptime_gating + self._f16(VOCALGORITHM_SAMPLING_INTERVAL))

        self._vocalgorithm__mean_variance_estimator___sigmoid__set_parameters(self._f16(1.), self._f16(
            VOCALGORITHM_INITI_DURATION_MEAN), self._f16(VOCALGORITHM_INITI_TRANSITION_MEAN))
        sigmoid_gamma_mean = self._vocalgorithm__mean_variance_estimator___sigmoid__process(
            self.params.m_mean_variance_estimator_uptime_gamma)
        gamma_mean = (self.params.m_mean_variance_estimator_gamma + (self._fix16_mul(
            (self.params.m_mean_variance_estimator_gamma_initial_mean - self.params.m_mean_variance_estimator_gamma), sigmoid_gamma_mean)))
        gating_threshold_mean = (self._f16(VOCALGORITHM_GATING_THRESHOLD)
                                 + (self._fix16_mul(self._f16((VOCALGORITHM_GATING_THRESHOLD_INITIAL - VOCALGORITHM_GATING_THRESHOLD)),
                                                    self._vocalgorithm__mean_variance_estimator___sigmoid__process(self.params.m_mean_variance_estimator_uptime_gating))))
        self._vocalgorithm__mean_variance_estimator___sigmoid__set_parameters(self._f16(
            1.), gating_threshold_mean, self._f16(VOCALGORITHM_GATING_THRESHOLD_TRANSITION))

        sigmoid_gating_mean = self._vocalgorithm__mean_variance_estimator___sigmoid__process(
            voc_index_from_prior)
        self.params.m_mean_variance_estimator_gamma_mean = (
            self._fix16_mul(sigmoid_gating_mean, gamma_mean))

        self._vocalgorithm__mean_variance_estimator___sigmoid__set_parameters(self._f16(1.), self._f16(
            VOCALGORITHM_INITI_DURATION_VARIANCE), self._f16(VOCALGORITHM_INITI_TRANSITION_VARIANCE))

        sigmoid_gamma_variance = self._vocalgorithm__mean_variance_estimator___sigmoid__process(
            self.params.m_mean_variance_estimator_uptime_gamma)

        gamma_variance = (self.params.m_mean_variance_estimator_gamma +
                          (self._fix16_mul((self.params.m_mean_variance_estimator_gamma_initial_variance
                                            - self.params.m_mean_variance_estimator_gamma),
                                           (sigmoid_gamma_variance - sigmoid_gamma_mean))))

        gating_threshold_variance = (self._f16(VOCALGORITHM_GATING_THRESHOLD)
                                     + (self._fix16_mul(self._f16((VOCALGORITHM_GATING_THRESHOLD_INITIAL - VOCALGORITHM_GATING_THRESHOLD)),
                                                        self._vocalgorithm__mean_variance_estimator___sigmoid__process(self.params.m_mean_variance_estimator_uptime_gating))))

        self._vocalgorithm__mean_variance_estimator___sigmoid__set_parameters(self._f16(
            1.), gating_threshold_variance, self._f16(VOCALGORITHM_GATING_THRESHOLD_TRANSITION))

        sigmoid_gating_variance = self._vocalgorithm__mean_variance_estimator___sigmoid__process(
            voc_index_from_prior)

        self.params.m_mean_variance_estimator__gamma_variance = (
            self._fix16_mul(sigmoid_gating_variance, gamma_variance))

        self.params.m_mean_variance_estimator_gating_duration_minutes = (self.params.m_mean_variance_estimator_gating_duration_minutes
                                                                         + (self._fix16_mul(self._f16((VOCALGORITHM_SAMPLING_INTERVAL / 60.)),
                                                                                            ((self._fix16_mul((self._f16(1.) - sigmoid_gating_mean),
                                                                                                              self._f16((1. + VOCALGORITHM_GATING_MAX_RATIO))))
                                                                                             - self._f16(VOCALGORITHM_GATING_MAX_RATIO)))))

        if ((self.params.m_mean_variance_estimator_gating_duration_minutes < self._f16(0.))):
            self.params.m_mean_variance_estimator_gating_duration_minutes = self._f16(
                0.)

        if ((self.params.m_mean_variance_estimator_gating_duration_minutes > self.params.m_mean_variance_estimator_gating_max_duration_minutes)):
            self.params.m_mean_variance_estimator_uptime_gating = self._f16(0.)

    def _vocalgorithm__mean_variance_estimator__process(self, sraw, voc_index_from_prior):
        if ((self.params.m_mean_variance_estimator_initialized == 0)):
            self.params.m_mean_variance_estimator_initialized = 1
            self.params.m_mean_variance_estimator_sraw_offset = sraw
            self.params.m_mean_variance_estimator_mean = self._f16(0.)
        else:
            if (((self.params.m_mean_variance_estimator_mean >= self._f16(100.)) or (self.params.m_mean_variance_estimator_mean <= self._f16(-100.)))):
                self.params.m_mean_variance_estimator_sraw_offset = (
                    self.params.m_mean_variance_estimator_sraw_offset + self.params.m_mean_variance_estimator_mean)
                self.params.m_mean_variance_estimator_mean = self._f16(0.)

            sraw = (sraw - self.params.m_mean_variance_estimator_sraw_offset)
            self._vocalgorithm__mean_variance_estimator___calculate_gamma(
                voc_index_from_prior)
            delta_sgp = (self._fix16_div((sraw - self.params.m_mean_variance_estimator_mean),
                                         self._f16(VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING)))
            if ((delta_sgp < self._f16(0.))):
                c = (self.params.m_mean_variance_estimator_std - delta_sgp)
            else:
                c = (self.params.m_mean_variance_estimator_std + delta_sgp)
            additional_scaling = self._f16(1.)
            if ((c > self._f16(1440.))):
                additional_scaling = self._f16(4.)
            self.params.m_mean_variance_estimator_std = self._fix16_mul(self._fix16_sqrt((self._fix16_mul(additional_scaling,
                                                                                                          (self._f16(VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING) - self.params.m_mean_variance_estimator__gamma_variance)))),
                                                                        self._fix16_sqrt(((self._fix16_mul(self.params.m_mean_variance_estimator_std,
                                                                                                           (self._fix16_div(self.params.m_mean_variance_estimator_std,
                                                                                                                            (self._fix16_mul(self._f16(VOCALGORITHM_MEAN_VARIANCE_ESTIMATOR__GAMMA_SCALING), additional_scaling))))))
                                                                                          + (self._fix16_mul((self._fix16_div((self._fix16_mul(self.params.m_mean_variance_estimator__gamma_variance, delta_sgp)), additional_scaling)), delta_sgp)))))
            self.params.m_mean_variance_estimator_mean = (self.params.m_mean_variance_estimator_mean + (
                self._fix16_mul(self.params.m_mean_variance_estimator_gamma_mean, delta_sgp)))

    def _vocalgorithm__mean_variance_estimator___sigmoid__init(self):
        self._vocalgorithm__mean_variance_estimator___sigmoid__set_parameters(
            self._f16(0.), self._f16(0.), self._f16(0.))

    def _vocalgorithm__mean_variance_estimator___sigmoid__set_parameters(self, L, X0, K):
        self.params.m_mean_variance_estimator_sigmoid_l = L
        self.params.m_mean_variance_estimator_sigmoid_k = K
        self.params.m_mean_variance_estimator_sigmoid_x0 = X0

    def _vocalgorithm__mean_variance_estimator___sigmoid__process(self, sample):
        x = (self._fix16_mul(self.params.m_mean_variance_estimator_sigmoid_k,
                             (sample - self.params.m_mean_variance_estimator_sigmoid_x0)))
        if ((x < self._f16(-50.))):
            return self.params.m_mean_variance_estimator_sigmoid_l
        elif ((x > self._f16(50.))):
            return self._f16(0.)
        else:
            return (self._fix16_div(self.params.m_mean_variance_estimator_sigmoid_l, (self._f16(1.) + self._fix16_exp(x))))

    def _vocalgorithm__mox_model__init(self):
        self._vocalgorithm__mox_model__set_parameters(
            self._f16(1.), self._f16(0.))

    def _vocalgorithm__mox_model__set_parameters(self, SRAW_STD, SRAW_MEAN):
        self.params.m_mox_model_sraw_std = SRAW_STD
        self.params.m_mox_model_sraw_mean = SRAW_MEAN

    def _vocalgorithm__mox_model__process(self, sraw):
        return (self._fix16_mul((self._fix16_div((sraw - self.params.m_mox_model_sraw_mean), (-(self.params.m_mox_model_sraw_std + self._f16(VOCALGORITHM_SRAW_STD_BONUS))))), self._f16(VOCALGORITHM_VOC_INDEX_GAIN)))

    def _vocalgorithm__sigmoid_scaled__init(self):
        self._vocalgorithm__sigmoid_scaled__set_parameters(self._f16(0.))

    def _vocalgorithm__sigmoid_scaled__set_parameters(self, offset):
        self.params.m_sigmoid_scaled_offset = offset

    def _vocalgorithm__sigmoid_scaled__process(self, sample):
        x = (self._fix16_mul(self._f16(VOCALGORITHM_SIGMOID_K),
                             (sample - self._f16(VOCALGORITHM_SIGMOID_X0))))
        if ((x < self._f16(-50.))):
            return self._f16(VOCALGORITHM_SIGMOID_L)
        elif ((x > self._f16(50.))):
            return self._f16(0.)
        else:
            if ((sample >= self._f16(0.))):
                shift = (self._fix16_div((self._f16(VOCALGORITHM_SIGMOID_L) - (self._fix16_mul(
                    self._f16(5.), self.params.m_sigmoid_scaled_offset))), self._f16(4.)))
                return ((self._fix16_div((self._f16(VOCALGORITHM_SIGMOID_L) + shift), (self._f16(1.) + self._fix16_exp(x)))) - shift)
            else:
                return (self._fix16_mul((self._fix16_div(self.params.m_sigmoid_scaled_offset, self._f16(VOCALGORITHM_VOC_INDEX_OFFSET_DEFAULT))),
                                        (self._fix16_div(self._f16(VOCALGORITHM_SIGMOID_L), (self._f16(1.) + self._fix16_exp(x))))))

    def _vocalgorithm__adaptive_lowpass__init(self):
        self._vocalgorithm__adaptive_lowpass__set_parameters()

    def _vocalgorithm__adaptive_lowpass__set_parameters(self):
        self.params.m_adaptive_lowpass_a1 = self._f16(
            (VOCALGORITHM_SAMPLING_INTERVAL / (VOCALGORITHM_LP_TAU_FAST + VOCALGORITHM_SAMPLING_INTERVAL)))
        self.params.m_adaptive_lowpass_a2 = self._f16(
            (VOCALGORITHM_SAMPLING_INTERVAL / (VOCALGORITHM_LP_TAU_SLOW + VOCALGORITHM_SAMPLING_INTERVAL)))
        self.params.m_adaptive_lowpass_initialized = 0

    def _vocalgorithm__adaptive_lowpass__process(self, sample):
        if ((self.params.m_adaptive_lowpass_initialized == 0)):
            self.params.m_adaptive_lowpass_x1 = sample
            self.params.m_adaptive_lowpass_x2 = sample
            self.params.m_adaptive_lowpass_x3 = sample
            self.params.m_adaptive_lowpass_initialized = 1
        self.params.m_adaptive_lowpass_x1 = ((self._fix16_mul((self._f16(1.) - self.params.m_adaptive_lowpass_a1),
                                                              self.params.m_adaptive_lowpass_x1)) + (self._fix16_mul(self.params.m_adaptive_lowpass_a1, sample)))

        self.params.m_adaptive_lowpass_x2 = ((self._fix16_mul((self._f16(1.) - self.params.m_adaptive_lowpass_a2),
                                                              self.params.m_adaptive_lowpass_x2)) + (self._fix16_mul(self.params.m_adaptive_lowpass_a2, sample)))

        abs_delta = (self.params.m_adaptive_lowpass_x1 -
                     self.params.m_adaptive_lowpass_x2)

        if ((abs_delta < self._f16(0.))):
            abs_delta = (-abs_delta)
        F1 = self._fix16_exp(
            (self._fix16_mul(self._f16(VOCALGORITHM_LP_ALPHA), abs_delta)))
        tau_a = ((self._fix16_mul(self._f16((VOCALGORITHM_LP_TAU_SLOW -
                                             VOCALGORITHM_LP_TAU_FAST)), F1)) + self._f16(VOCALGORITHM_LP_TAU_FAST))
        a3 = (self._fix16_div(self._f16(VOCALGORITHM_SAMPLING_INTERVAL),
                              (self._f16(VOCALGORITHM_SAMPLING_INTERVAL) + tau_a)))
        self.params.m_adaptive_lowpass_x3 = ((self._fix16_mul((self._f16(
            1.) - a3), self.params.m_adaptive_lowpass_x3)) + (self._fix16_mul(a3, sample)))
        return self.params.m_adaptive_lowpass_x3


__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_SGP40.git"

_WORD_LEN = 2

# no point in generating this each time
# Generated from temp 25c, humidity 50%
_READ_CMD = b"\x26\x0F\x80\x00\xA2\x66\x66\x93"


class SGP40:
    """
    Class to use the SGP40 Air Quality Sensor Breakout

    :param int address: The I2C address of the device. Defaults to :const:`0x59`


    **Quickstart: Importing and using the SGP40 temperature sensor**

        Here is one way of importing the `SGP40` class so you can use it with the name ``sgp``.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            import adafruit_sgp40
            # If you have a temperature sensor, like the bme280, import that here as well
            # import adafruit_bme280

        Once this is done you can define your `board.I2C` object and define your sensor object

        .. code-block:: python

            i2c = board.I2C()  # uses board.SCL and board.SDA
            sgp = adafruit_sgp40.SGP40(i2c)
            # And if you have a temp/humidity sensor, define the sensor here as well
            # bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

        Now you have access to the raw gas value using the :attr:`raw` attribute.
        And with a temperature and humidity value, you can access the class function
        :meth:`measure_raw` for a humidity compensated raw reading

        .. code-block:: python

            raw_gas_value = sgp.raw
            # Lets quickly grab the humidity and temperature
            # temperature = bme280.temperature
            # humidity = bme280.relative_humidity
            # compensated_raw_gas = sgp.measure_raw(temperature=temperature,
            # relative_humidity=humidity)
            # temperature = temperature, relative_humidity = humidity)



    .. note::
        The operational range of temperatures for the SGP40 is -10 to 50 degrees Celsius
        and the operational range of relative humidity for the SGP40 is 0 to 90 %
        (assuming that humidity is non-condensing).

        Humidity compensation is further optimized for a subset of the temperature
        and relative humidity readings. See Figure 3 of the Sensirion datasheet for
        the SGP40. At 25 degrees Celsius, the optimal range for relative humidity is 8% to 90%.
        At 50% relative humidity, the optimal range for temperature is -7 to 42 degrees Celsius.

        Prolonged exposures outside of these ranges may reduce sensor performance, and
        the sensor must not be exposed towards condensing conditions at any time.

        For more information see:
        https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9_Gas_Sensors/Datasheets/Sensirion_Gas_Sensors_Datasheet_SGP40.pdf
        and
        https://learn.adafruit.com/adafruit-sgp40

    """

    def __init__(self, i2c, address=0x59):
        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        self._command_buffer = bytearray(2)
        self._measure_command = _READ_CMD
        self._voc_algorithm = DFRobot_VOCAlgorithm()

        self.initialize()

    def initialize(self):
        """Reset the sensor to it's initial unconfigured state and configure it with sensible
        defaults so it can be used"""
        # check serial number
        self._command_buffer[0] = 0x36
        self._command_buffer[1] = 0x82
        serialnumber = self._read_word_from_command(3)

        if serialnumber[0] != 0x0000:
            raise RuntimeError("Serial number does not match")

        # Check feature set
        self._command_buffer[0] = 0x20
        self._command_buffer[1] = 0x2F
        featureset = self._read_word_from_command()
        if featureset[0] != 0x3220:

            raise RuntimeError("Feature set does not match: %s" %
                               hex(featureset[0]))

        self._voc_algorithm.vocalgorithm_init()

        # Self Test
        self._command_buffer[0] = 0x28
        self._command_buffer[1] = 0x0E
        self_test = self._read_word_from_command(delay_ms=250)
        if self_test[0] != 0xD400:
            raise RuntimeError("Self test failed")
        self._reset()

    def _reset(self):
        # This is a general call Reset. Several sensors may see this and it doesn't appear to
        # ACK before resetting
        self._command_buffer[0] = 0x00
        self._command_buffer[1] = 0x06
        try:
            self._read_word_from_command(delay_ms=50)
        except OSError:
            # Got expected OSError from reset
            pass
        sleep(1)

    @staticmethod
    def _celsius_to_ticks(temperature):
        """
        Converts Temperature in Celsius to 'ticks' which are an input parameter
        the sgp40 can use

        Temperature to Ticks : From SGP40 Datasheet Table 10
        temp (C)    | Hex Code (Check Sum/CRC Hex Code)
            25      | 0x6666   (CRC 0x93)
            -45     | 0x0000   (CRC 0x81)
            130     | 0xFFFF   (CRC 0xAC)

        """
        temp_ticks = int(((temperature + 45) * 65535) / 175) & 0xFFFF
        least_sig_temp_ticks = temp_ticks & 0xFF
        most_sig_temp_ticks = (temp_ticks >> 8) & 0xFF

        return [most_sig_temp_ticks, least_sig_temp_ticks]

    @staticmethod
    def _relative_humidity_to_ticks(humidity):
        """
        Converts Relative Humidity in % to 'ticks' which are  an input parameter
        the sgp40 can use

        Relative Humidity to Ticks : From SGP40 Datasheet Table 10
        Humidity (%) | Hex Code (Check Sum/CRC Hex Code)
            50       | 0x8000   (CRC 0xA2)
            0        | 0x0000   (CRC 0x81)
            100      | 0xFFFF   (CRC 0xAC)

        """
        humidity_ticks = int((humidity * 65535) / 100 + 0.5) & 0xFFFF
        least_sig_rhumidity_ticks = humidity_ticks & 0xFF
        most_sig_rhumidity_ticks = (humidity_ticks >> 8) & 0xFF

        return [most_sig_rhumidity_ticks, least_sig_rhumidity_ticks]

    @property
    def raw(self):
        """The raw gas value"""
        # recycle a single buffer
        self._command_buffer = self._measure_command
        read_value = self._read_word_from_command(delay_ms=250)
        self._command_buffer = bytearray(2)
        return read_value[0]

    def measure_raw(self, temperature=25, relative_humidity=50):
        """
        A humidity and temperature compensated raw gas value which helps
        address fluctuations in readings due to changing humidity.


        :param float temperature: The temperature in degrees Celsius, defaults
                                     to :const:`25`
        :param float relative_humidity: The relative humidity in percentage, defaults
                                     to :const:`50`

        The raw gas value adjusted for the current temperature (c) and humidity (%)
        """
        # recycle a single buffer
        _compensated_read_cmd = [0x26, 0x0F]
        humidity_ticks = self._relative_humidity_to_ticks(relative_humidity)
        humidity_ticks.append(self._generate_crc(humidity_ticks))
        temp_ticks = self._celsius_to_ticks(temperature)
        temp_ticks.append(self._generate_crc(temp_ticks))
        _cmd = _compensated_read_cmd + humidity_ticks + temp_ticks
        self._measure_command = bytearray(_cmd)
        return self.raw

    def measure_index(self, temperature=25, relative_humidity=50):
        """ Measure VOC index after humidity compensation
        :param float temperature: The temperature in degrees Celsius, defaults
                                     to :const:`25`
        :param float relative_humidity: The relative humidity in percentage, defaults
                                     to :const:`50`
        :note  VOC index can indicate the quality of the air directly. The larger the value, the worse the air quality.
        :note    0-100,no need to ventilate, purify
        :note    100-200,no need to ventilate, purify
        :note    200-400,ventilate, purify
        :note    00-500,ventilate, purify intensely
        :return int The VOC index measured, ranged from 0 to 500
        """
        raw = self.measure_raw(temperature, relative_humidity)
        if raw < 0:
            return -1
        else:
            vocIndex = self._voc_algorithm.vocalgorithm_process(raw)
            return vocIndex

    def _read_word_from_command(
        self,
        delay_ms=10,
        readlen=1,
    ):
        """_read_word_from_command - send a given command code and read the result back

        Args:
            delay_ms (int, optional): The delay between write and read, in milliseconds.
                Defaults to 10ms
            readlen (int, optional): The number of bytes to read. Defaults to 1.
        """
        # TODO: Take 2-byte command as int (0x280E, 0x0006) and packinto command buffer

        with self.i2c_device as i2c:
            i2c.write(self._command_buffer)

        sleep(round(delay_ms * 0.001, 3))

        if readlen is None:
            return None
        readdata_buffer = []

        # The number of bytes to read back, based on the number of words to read
        replylen = readlen * (_WORD_LEN + 1)
        # recycle buffer for read/write w/length
        replybuffer = bytearray(replylen)

        with self.i2c_device as i2c:
            i2c.readinto(replybuffer, end=replylen)

        for i in range(0, replylen, 3):
            if not self._check_crc8(replybuffer[i: i + 2], replybuffer[i + 2]):
                raise RuntimeError("CRC check failed while reading data")
            readdata_buffer.append(unpack_from(">H", replybuffer[i: i + 2])[0])

        return readdata_buffer

    def _check_crc8(self, crc_buffer, crc_value):
        """
        Checks that the 8 bit CRC Checksum value from the sensor matches the
        received data
        """
        return crc_value == self._generate_crc(crc_buffer)

    @staticmethod
    def _generate_crc(crc_buffer):
        """
        Generates an 8 bit CRC Checksum from the input buffer.

        This checksum algorithm is outlined in Table 7 of the SGP40 datasheet.

        Checksums are only generated for 2-byte data packets. Command codes already
        contain 3 bits of CRC and therefore do not need an added checksum.
        """
        crc = 0xFF
        for byte in crc_buffer:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (
                        crc << 1
                    ) ^ 0x31  # 0x31 is the Seed for SGP40's CRC polynomial
                else:
                    crc = crc << 1
        return crc & 0xFF  # Returns only bottom 8 bits
