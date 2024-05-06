
use pac::adc::regs::Cfgr2;
use stm32_metapac as pac;

use crate::gpio::{gpios, Pin};

pub fn configure_op_amps() {
    use pac::opamp::vals;

    // Turn on OPAMP + SYSCFG + COMP + VREFBUF clock
    pac::RCC.apb2enr().modify(|w| w.set_syscfgen(true));

    let amp1 = pac::OPAMP1;
    let amp2 = pac::OPAMP3;
    let amp3 = pac::OPAMP4;
    let amp4 = pac::OPAMP2;

    let gpios = gpios();
    gpios.PA3.set_as_analog();

    amp1.csr().write(|w| {
        w.set_opahsm(vals::Opahsm::HIGHSPEED);
        w.set_pga_gain(vals::PgaGain::GAIN16);
        w.set_vp_sel(vals::VpSel::VINP1); // VINP1 = PA3
        w.set_vm_sel(vals::VmSel::PGA);
        w.set_opaintoen(vals::Opaintoen::ADCCHANNEL); // ADC1_IN13
        w.set_opampen(true);
    });

    amp2.csr().write(|w| {
        w.set_opahsm(vals::Opahsm::HIGHSPEED);
        w.set_pga_gain(vals::PgaGain::GAIN16);
        w.set_vp_sel(vals::VpSel::VINP2); // VINP2 = PA1
        w.set_vm_sel(vals::VmSel::PGA);
        w.set_opaintoen(vals::Opaintoen::ADCCHANNEL); // ADC3_IN13
        w.set_opampen(true);
    });

    amp3.csr().write(|w| {
        w.set_opahsm(vals::Opahsm::HIGHSPEED);
        w.set_pga_gain(vals::PgaGain::GAIN16);
        w.set_vp_sel(vals::VpSel::VINP2); // VINP2 = PB11
        w.set_vm_sel(vals::VmSel::PGA);
        w.set_opaintoen(vals::Opaintoen::ADCCHANNEL); // ADC5_IN5
        w.set_opampen(true);
    });

    amp4.csr().write(|w| {
        w.set_opahsm(vals::Opahsm::HIGHSPEED);
        w.set_pga_gain(vals::PgaGain::GAIN16);
        w.set_vp_sel(vals::VpSel::VINP2); // VINP2 = PB0
        w.set_vm_sel(vals::VmSel::PGA);
        w.set_opaintoen(vals::Opaintoen::ADCCHANNEL); // ADC2_IN16
        w.set_opampen(true);
    });
}

fn configure_adc(adc: pac::adc::Adc, channel: usize)  {
    use pac::adc::vals;

    adc.cr().modify(|w| {
        w.set_deeppwd(false);
        w.set_advregen(true)
    });

    cortex_m::asm::delay(4000000);

    while adc.cr().read().aden() {}

    adc.cr().modify(|w| w.set_adcaldif(vals::Adcaldif::SINGLEENDED));
    adc.cr().modify(|w| w.set_adcal(true));

    // Wait for calibration to complete
    while adc.cr().read().adcal() {}

    // Clear ADRDY IRQ
    adc.isr().write(|w| w.set_adrdy(true));
    // Enable
    adc.cr().modify(|w| w.set_aden(true));

    // Wait for ADRDY signal
    while !adc.isr().read().adrdy() {}
    // Clear the flag again
    adc.isr().write(|w| w.set_adrdy(true));

    adc.cfgr().modify(|w| {
        // Triggered mode; not continuous
        w.set_cont(false);
        // Trigger on rising edge of external trigger
        w.set_exten(vals::Exten::RISINGEDGE);
        // TIM1 TRGO2 as trigger source
        w.set_extsel(10);

        // This combined with the TROVS bit, mean each trigger triggers one oversampler
        // accumulation, and every N triggers lead to a completed oversampled conversion
        w.set_discen(true);

        // Ignore overruns and overwrite old data
        w.set_ovrmod(vals::Ovrmod::OVERWRITE);
    });


    // metapac cfgr2 register definition is broken, so set oversampling settings manually\
    // OVSR = 6: 128x oversampling
    // OVSS = 3: shift left 3 bits
    //adc.cfgr2().write_value(Cfgr2((3 << 5) | (6 << 2) | 1));
    adc.cfgr2().write_value(Cfgr2((0 << 5) | (3 << 2) | 1));

    adc.cfgr2().modify(|w| w.set_trovs(vals::Trovs::TRIGGERED));

    let sample_time = vals::SampleTime::CYCLES1_5;

    if channel <= 9 {
        adc.smpr(0).modify(|w| w.set_smp(channel as _, sample_time));
    } else {
        adc.smpr(1).modify(|w| w.set_smp((channel - 10) as _, sample_time));
    }

    adc.sqr1().modify(|w| {
        w.set_l(0);
        w.set_sq(0, channel as _);
    });

}

pub fn configure_adcs() {
    use pac::adccommon::vals;

    pac::RCC.ccipr().modify(|w| {
        w.set_adc12sel(pac::rcc::vals::Adcsel::SYS);
        w.set_adc345sel(pac::rcc::vals::Adcsel::SYS);
    });

    // Turn on peripheral clocks and reset
    pac::RCC.ahb2enr().modify(|w| {
        w.set_adc12en(true);
        w.set_adc345en(true);
    });

    pac::RCC.ahb2rstr().modify(|w| {
        w.set_adc12rst(true);
        w.set_adc345rst(true);
    });

    pac::RCC.ahb2rstr().modify(|w| {
        w.set_adc12rst(false);
        w.set_adc345rst(false);
    });

    // Common ADC config
    pac::ADC_COMMON.ccr().modify(|w| {
        // Prescaler of 4, to yeild ADC clock of 128/4 = 32MHz
        w.set_presc(vals::Presc::DIV16);
    });

    // stm32_metapac seems to be broken wrt the split common regs
    pub const ADC345_COMMON: pac::adccommon::AdcCommon = unsafe { pac::adccommon::AdcCommon::from_ptr(0x5000_0700 as usize as _) };
    // Common ADC config
    ADC345_COMMON.ccr().modify(|w| {
        // Prescaler of 4, to yeild ADC clock of 128/4 = 32MHz
        w.set_presc(vals::Presc::DIV16);
    });

    const ADC1_CHAN: usize = 13;
    const ADC2_CHAN: usize = 16;
    const ADC3_CHAN: usize = 13;
    const ADC5_CHAN: usize = 5;

    configure_adc(pac::ADC1, ADC1_CHAN);
    configure_adc(pac::ADC2, ADC2_CHAN);
    configure_adc(pac::ADC3, ADC3_CHAN);
    configure_adc(pac::ADC4, 0);
    configure_adc(pac::ADC5, ADC5_CHAN);

    start_conversion(pac::ADC1);
    start_conversion(pac::ADC2);
    start_conversion(pac::ADC3);
    start_conversion(pac::ADC5);
}

pub fn trim_opamps() {
    let mut finished = [false; 4];
    let mut trim = 0;

    // Ordered as the channels are ordered
    let opamps = [pac::OPAMP1, pac::OPAMP3, pac::OPAMP4, pac::OPAMP2];

    for i in 0..4 {
        opamps[i].csr().modify(|w| {
            w.set_calsel(pac::opamp::vals::Calsel::PERCENT90);
            w.set_usertrim(pac::opamp::vals::Usertrim::USER);
            w.set_calon(true);
        });
    }

    cortex_m::asm::delay(256_000);

    read_isense();

    while finished != [true; 4] && trim < 32 {
        for i in 0..4 {
            if !finished[i] {
                opamps[i].csr().modify(|w| w.set_trimoffsetn(trim))
            }
        }
        // Need as much as 2ms stabilization time
        cortex_m::asm::delay(256_000);

        let voltage = read_isense();

        defmt::info!("N {}: {} {} {} {}", trim, voltage[0], voltage[1], voltage[2], voltage[3]);

        for i in 0..4 {
            if voltage[i] < 2048 && !finished[i] {
                finished[i] = true;
                defmt::info!("Opamp channel {} trimmed at {}", i, trim);
            }
        }

        trim += 1;
    }

    for i in 0..4 {
        opamps[i].csr().modify(|w| {
            w.set_calsel(pac::opamp::vals::Calsel::PERCENT10);
        });
    }
    finished = [false; 4];
    trim = 0;

    while finished != [true; 4] && trim < 32 {
        for i in 0..4 {
            if !finished[i] {
                opamps[i].csr().modify(|w| w.set_trimoffsetp(trim))
            }
        }
        // Need as much as 2ms stabilization time
        cortex_m::asm::delay(256_000);

        let voltage = read_isense();

        defmt::info!("P {}: {} {} {} {}", trim, voltage[0], voltage[1], voltage[2], voltage[3]);

        for i in 0..4 {
            if voltage[i] < 2048 && !finished[i] {
                finished[i] = true;
                defmt::info!("Opamp channel {} trimmed at {}", i, trim);
            }
        }

        trim += 1;
    }


    for i in 0..4 {
        opamps[i].csr().modify(|w| {
            w.set_calon(false);
        });
    }
}

fn start_conversion(adc: pac::adc::Adc) {
    // Clear end-of-conversion ISR flags
    adc.isr().write(|w| {
        w.set_eoc(true);
        w.set_eos(true);
    });

    // Trigger conversion
    adc.cr().modify(|w| {
        w.set_adstart(true);
    });
}

fn wait_for_conversion(adc: pac::adc::Adc) {
    while !adc.isr().read().eos() {}
}

fn read_adc_result(adc: pac::adc::Adc) -> u16 {
    adc.dr().read().0 as u16
}


pub fn read_isense() -> [u16; 4] {
    use pac::opamp::vals;



    // ADC is triggered by timer every PWM cycle; just read the latest result
    [
        read_adc_result(pac::ADC1),
        read_adc_result(pac::ADC3),
        read_adc_result(pac::ADC5),
        read_adc_result(pac::ADC2),
    ]
}

// pub fn read_isense() -> [u16; 4] {
//     use pac::opamp::vals;

//     start_conversion(pac::ADC1);
//     start_conversion(pac::ADC2);
//     start_conversion(pac::ADC3);
//     start_conversion(pac::ADC4);
//     start_conversion(pac::ADC5);

//     // They are all on same clock and same config, so should complete in same time
//     wait_for_conversion(pac::ADC5);

//     [
//         read_adc_result(pac::ADC1),
//         read_adc_result(pac::ADC3),
//         read_adc_result(pac::ADC5),
//         read_adc_result(pac::ADC2),
//     ]
// }