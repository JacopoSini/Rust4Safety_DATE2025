#![no_std]
#![no_main]

use core::panic::PanicInfo;

#[no_mangle] //ensures that the name is kept exactly as it is written in the source code
pub static mut ERR_CODE : i32 = 0;
#[no_mangle]
pub static mut COUNT : i32 = 0;
#[no_mangle]
pub static mut ADJUST_VALUE : i32 = 0;
#[no_mangle]
pub static mut RETURN_VAL : i32 = 0;
#[no_mangle]
pub static mut SUM : i32 = 0;
#[no_mangle]
pub static mut SIGNATURE : i32 = 145;
#[no_mangle]
pub static mut terminated: u64 = 0;
#[no_mangle]
pub static mut y : f64 = 0.0;

// Arrays
#[no_mangle]
static SUB_RAN_PREV_VAL: [i32; 39] = [98, 21, 59, 60, 7, 39, 70, 74, 23, 20, 35, 66, 18, 92, 13, 67, 58, 64, 57, 42, 99, 55, 77, 17, 33, 73, 1, 14, 83, 68, 85, 909, 808, 61, 49, 100, 678, 40, 98];
#[no_mangle]
static CTS: [i32; 39] = [47, 65, 29, 34, 82, 95, 123, 90, 567, 22, 987, 93, 456, 345, 37, 789, 62, 3, 321, 543, 43, 765, 556, 667, 888, 111, 52, 86, 333, 96, 444, 71, 80, 999, 94, 48, 9, 555, 47];
#[no_mangle]
static SUM_BB: [i32; 39] = [50, 0, 0, 0, 50, 0, 0, 0, 0, 0, 36, 43, 0, 0, 156, 0, 0, 179, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 261, 0, 135, 0, 0, 50];

#[no_mangle]
pub extern "C" fn main() -> () {
    let mut controller = RTModelController::default();
    controller.initialize();
    // Example: Running the step function in a loop
    let  mut level_cm: f64 = 130.0; //Initial condition
    let out_flow_cm3_s: f64 = 1.0;
    //println!("Level, hfp, lfp, error, status, cfcerror");
    for i in 0..5 {
        //Set inputs
        controller.u.level_cm = level_cm;
        controller.u.out_flow_cm3_s = out_flow_cm3_s;

        controller.controller_step();
        unsafe{SIGNATURE = 145;} //Reset signature status
        //Read outputs
        let hfp = controller.y.hfp_do;
        let lfp = controller.y.lfp_do;
        let error = controller.y.error;
        let status = controller.dw.is_controller;

        level_cm = level_cm + ((1.0 * if(controller.y.lfp_do) {1.0} else{0.0}) + (3.0 * if(controller.y.lfp_do) {1.0} else{0.0}) - (0.5 * out_flow_cm3_s))/10.0;

        unsafe{ /*CFCS.*/y = level_cm; }
        //Only when run on PCs
        //let ERR_CODE_local = unsafe {/*CFCS.*/ERR_CODE};
        //println!("{level_cm}, {hfp}, {lfp}, {error}, {status}, {ERR_CODE_local}");
    }
    controller.terminate();
    unsafe{ /*CFCS.*/terminated = 1; }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

fn abs(val : f64) -> f64 {
    if (val < 0.0) {return -val;}
    return val;
}

fn UpdSigBegin(current_mask: usize){
    if(current_mask < CTS.len()){
        let prev_val = SUB_RAN_PREV_VAL[current_mask];
        unsafe{
            let current_signature = SIGNATURE;
            SIGNATURE = current_signature - prev_val;
        }
    }
    else {
        unsafe{
            ERR_CODE = 1000;
        }
    }
}

fn CheckSig(current_mask: usize){
    unsafe{
        let current_signature = SIGNATURE;
        if (current_mask < CTS.len() && current_signature != CTS[current_mask] as i32){
            ERR_CODE = current_mask as i32;
            
        }
    }
}

fn AddRand(rand: i32){
    unsafe{
        let current_signature = SIGNATURE;
        SIGNATURE = current_signature + rand;
    }
}

fn UpdSigEnd(current_mask: usize, successor_mask: usize){
    if(current_mask < CTS.len() && successor_mask < CTS.len()){
        let adjust_value = -(CTS[current_mask] + SUM_BB[current_mask] as i32) + (CTS[successor_mask] + SUB_RAN_PREV_VAL[successor_mask]);
        unsafe{
            let current_signature = SIGNATURE;
            SIGNATURE = current_signature + adjust_value;
        }
    }
    else {
        unsafe {
            ERR_CODE = 1000;
        }
    }
}

// Constants for Chart
const CONTROLLER_IN_HFP_FAILURE: u8 = 1;
const CONTROLLER_IN_HFP_ON: u8 = 2;
const CONTROLLER_IN_LFP_FAILURE: u8 = 4;
const CONTROLLER_IN_LFP_ON: u8 = 8;
const CONTROLLER_IN_NO_ERR: u8 = 16;
const CONTROLLER_IN_OFF: u8 = 32;

// Block states (default storage)
struct DWController {
    is_active_c3_controller: u8,
    is_controller: u8,
    is_monitor: u8,
    temporal_counter_i1: u8,
    ud_dstate: f64,
}

impl Default for DWController {
    fn default() -> Self {
        DWController {
            is_active_c3_controller: 0,
            is_controller: CONTROLLER_IN_OFF,
            is_monitor: CONTROLLER_IN_NO_ERR,
            temporal_counter_i1: 0,
            ud_dstate: 0.0,
        }
    }
}

// External inputs
struct ExtUController {
    level_cm: f64,
    out_flow_cm3_s: f64,
}

// External outputs
struct ExtYController {
    error: u8,
    hfp_do: bool,
    lfp_do: bool,
}

// Real-time model
struct RTModelController {
    dw: DWController,
    u: ExtUController,
    y: ExtYController,
}

impl Default for RTModelController {
    fn default() -> Self {
        RTModelController {
            dw: DWController::default(),
            u: ExtUController {
                level_cm: 0.0,
                out_flow_cm3_s: 0.0,
            },
            y: ExtYController {
                error: 0,
                hfp_do: false,
                lfp_do: false,
            },
        }
    }
}

impl RTModelController{
    fn controller_step(&mut self){
        UpdSigBegin(0);
        CheckSig(0);
        
                let rtb_Diff : f64;
        AddRand(5);// No sense in declarations?
                let rtb_TSamp : f64;
        AddRand(10);
                /* SampleTimeMath: '<S2>/TSamp' incorporates:
                *  Inport: '<Root>/Level_cm'
                *
                * About '<S2>/TSamp':
                *  y = u * K where K = 1 / ( w * Ts )
                */
        
                rtb_TSamp = self.u.level_cm * 10.0;
        AddRand(15);
                /* Sum: '<S2>/Diff' incorporates:
                *  UnitDelay: '<S2>/UD'
                *
                * Block description for '<S2>/Diff':
                *
                *  Add in CPU
                *
                * Block description for '<S2>/UD':
                *
                *  Store in Global RAM
                */
        
                rtb_Diff = rtb_TSamp - self.dw.ud_dstate;
        AddRand(20);
                /* Chart: '<Root>/Chart' incorporates:
                *  Inport: '<Root>/Level_cm'
                *  Inport: '<Root>/Out_Flow_cm3_s'
                *  Outport: '<Root>/Error'
                *  Outport: '<Root>/HFP_DO'
                *  Outport: '<Root>/LFP_DO'
                */
        
        UpdSigEnd(0,1);
        
        //BB1 YACCA test legal predecessor is 0
        //BB1 RACFED setting adjustvalue successor 2,3
        UpdSigBegin(1);
        
                if(self.dw.temporal_counter_i1 < 63){
        CheckSig(1);
        UpdSigEnd(1,2);
        
        //BB2 YACCA test legal predecessor is 1
        UpdSigBegin(2);
        CheckSig(2);
        
                    self.dw.temporal_counter_i1 += 1;
        
        UpdSigEnd(2,3);
                } else { //RACFED
        CheckSig(1);
        UpdSigEnd(1,3);            
                }
        
        
        //BB3 YACCA test legal predecessor are 2 or 1
        //BB3 RACFED setting adjustvalue successor 4,5
        
        UpdSigBegin(3);
        
                if self.dw.is_active_c3_controller == 0 {
        CheckSig(3);
        UpdSigEnd(3,4);
        
        //BB4 YACCA test legal predecessor is 3
        //BB4 RACFED setting adjustvalue successor 38
        //BB4 more than 2 instructions
        
        UpdSigBegin(4);
        CheckSig(4);
        
                    self.dw.is_active_c3_controller = 1;
        AddRand(2);  
                    self.dw.is_controller = CONTROLLER_IN_OFF;
        AddRand(3);   
                    /* Outport: '<Root>/HFP_DO' */
                    self.y.hfp_do = false;
        AddRand(4);   
                    /* Outport: '<Root>/LFP_DO' */
                    self.y.lfp_do = false;
        AddRand(5); 
        
                    self.dw.is_monitor = CONTROLLER_IN_NO_ERR;
        AddRand(6);  
                    /* Outport: '<Root>/Error' */
                    self.y.error = 0;
        
        AddRand(30);   
        
        UpdSigEnd(4,38);
        
        //BB5 YACCA test legal predecessor is 3
        //BB5 RACFED setting adjustvalue successor 6
        
                } else {
        
        CheckSig(3);
        UpdSigEnd(3,5);
        
        UpdSigBegin(5);
        CheckSig(5);
        UpdSigEnd(5,6);
        
        //BB6 YACCA test legal predecessor is 5
        //BB6 RACFED setting adjustvalue successor 13,19
        
        UpdSigBegin(6);
        
                    match self.dw.is_controller {
        
        //BB7 YACCA test legal predecessor is 6
        //BB7 RACFED setting adjustvalue successor 8,9,11
        
                        CONTROLLER_IN_HFP_ON => {
        CheckSig(6);
        UpdSigEnd(6,7);
        UpdSigBegin(7);
        
                            if ((rtb_Diff > 0.0) && (self.u.level_cm > 155.0)) || (self.y.error == 2) {
        CheckSig(7);
        UpdSigEnd(7,8);   
        
        //BB8   YACCA test legal predecessor is 7
        //BB8 RACFED setting adjustvalue successor 12
        UpdSigBegin(8);
        CheckSig(8);
                                self.dw.is_controller = CONTROLLER_IN_LFP_ON;
        
        UpdSigEnd(8,12);
        
        //BB9   YACCA test legal predecessor is 7
        //BB9  RACFED setting adjustvalue successor 10,11,12
        
                            } else if self.u.level_cm > 180.0 {
        CheckSig(7);
        UpdSigEnd(7,10);
        
        //BB10 YACCA test legal predecessor is 9
        //BB10 RACFED setting adjustvalue successor 12
        //BB10 more than 2 instructions
        UpdSigBegin(10);
        CheckSig(10);
                            self.dw.is_controller = CONTROLLER_IN_OFF;
        AddRand(11);
                            /* Outport: '<Root>/HFP_DO' */
                            self.y.hfp_do = false;
        AddRand(12);
                            /* Outport: '<Root>/LFP_DO' */
                            self.y.lfp_do = false;
        AddRand(13);
        
        UpdSigEnd(10,12);
        
        //BB11 YACCA test legal predecessor is 7
        //BB11 RACFED setting adjustvalue successor 12
        //BB11 more than 2 instructions
        
                            } else {
        CheckSig(7);
        UpdSigEnd(7,11);
        
        UpdSigBegin(11);
        CheckSig(11);
                                /* Outport: '<Root>/HFP_DO' */
                                self.y.hfp_do = true;
        AddRand(21);
                                /* Outport: '<Root>/LFP_DO' */
                                self.y.lfp_do = false;
        AddRand(22);
        UpdSigEnd(11,12);
                            }
        
        //BB12 YACCA test legal predecessor are 11,10,8
        //BB12 RACFED setting adjustvalue successor 24
        UpdSigBegin(12);
        CheckSig(12);
        UpdSigEnd(12,24);
        
        //for the BB24
        UpdSigBegin(24);
        CheckSig(24);
                        } //corresponds to the C break;
        
        //BB13 YACCA test legal predecessor 6
        //BB13 RACFED setting adjustvalue successor 14,15,17
                        CONTROLLER_IN_LFP_ON => {
        CheckSig(6);
        UpdSigEnd(6,13);
        UpdSigBegin(13);       
        
                            if (rtb_Diff > 0.0) && (self.u.level_cm > 180.0) {
        CheckSig(13);
        UpdSigEnd(13,14);
        
        //BB14 YACCA test legal predecessor 13
        //BB14 RACFED setting adjustvalue successor 18
        //BB14 more than 2 instructions
        
        UpdSigBegin(14);
        CheckSig(14);
                                self.dw.is_controller = CONTROLLER_IN_OFF;
        AddRand(51);
                                /* Outport: '<Root>/HFP_DO' */
                                self.y.hfp_do = false;
        AddRand(52);
                                /* Outport: '<Root>/LFP_DO' */
                                self.y.lfp_do = false;
        AddRand(53);
        UpdSigEnd(14,18);
        
        //BB15 YACCA test legal predecessor is 13
        //BB15 RACFED setting adjustvalue successor 16
                            } else if ((rtb_Diff < 0.0) && (self.u.level_cm < 150.0) && 
                                        (self.y.error != 2)) || (self.y.error == 1) {
        CheckSig(13);                        
        UpdSigEnd(13,15);
        
        UpdSigBegin(15);
        CheckSig(15);
        UpdSigEnd(15,16);
        
        //BB16 YACCA test legal predecessor is 15
        //BB16 RACFED setting adjustvalue successor 18
        UpdSigBegin(16);
        CheckSig(16); 
        
                                self.dw.is_controller = CONTROLLER_IN_HFP_ON;
        UpdSigEnd(16,18);//end of the if-else
        
        //BB17 YACCA test legal predecessor is 13
        //BB17 RACFED setting adjustvalue successor 18
        //BB17 more than 2 instructions
        
                            } else {
        CheckSig(13);
        UpdSigEnd(13,17);
        UpdSigBegin(17);
        CheckSig(17);  
        
                                /* Outport: '<Root>/HFP_DO' */
                                self.y.hfp_do = false;
        AddRand(90);
                                /* Outport: '<Root>/LFP_DO' */
                                self.y.lfp_do = true;
        AddRand(89);
        UpdSigEnd(17,18);//end of the if-else
                            }
        
        //BB18 YACCA test legal predecessor are 17 & 16 & 14 & 13
        //BB18 RACFED setting adjustvalue successor 24
        UpdSigBegin(18);
        CheckSig(18);
        UpdSigEnd(18,24);
        
        //for BB24
        UpdSigBegin(24);
        CheckSig(24);
                        } //corresponds to the C break;
        
        
        //BB19 YACCA test legal predecessor is 6
        //BB19 RACFED setting adjustvalue successor 20,23
                        _ => {
        CheckSig(6);
        UpdSigEnd(6,19);
        UpdSigBegin(19);
                            /* case IN_OFF: */
                            if (rtb_Diff < 0.0) && (self.u.level_cm < 150.0) {
        CheckSig(19);
        UpdSigEnd(19,20);
        //BB20 YACCA test legal predecessor is 19
        //BB20 RACFED setting adjustvalue successor 23
        UpdSigBegin(20);
        CheckSig(20);
                                self.dw.is_controller = CONTROLLER_IN_HFP_ON;
        UpdSigEnd(20,23);
        //BB21 YACCA test legal predecessor is 19
        //BB21 RACFED setting adjustvalue successor 23
        
                            } else if (rtb_Diff < 0.0) && (self.u.level_cm < 175.0) {
        CheckSig(19);
        UpdSigEnd(19,21);
        
        UpdSigBegin(21);
        CheckSig(21);
        UpdSigEnd(21,22);
        //BB22 YACCA test legal predecessor is 21
        //BB22 RACFED setting adjustvalue successor 23
        UpdSigBegin(22);
        CheckSig(22);
                                self.dw.is_controller = CONTROLLER_IN_LFP_ON;
        UpdSigEnd(22,23);
                            } else { //RACFED
        CheckSig(19);
        UpdSigEnd(19,23);
                            }
        
        //BB23 YACCA test legal predecessor are 22 & 21 & 20 & 19
        //BB23 RACFED setting adjustvalue successor 24
        
        UpdSigBegin(23);
        CheckSig(23);
        UpdSigEnd(23,24);
        //for BB24
        UpdSigBegin(24);
        CheckSig(24);
                        } //corresponds to the C break;
        //BB24
        //BB24 RACFED setting adjustvalue successor 29
                    }
        
                    match self.dw.is_monitor {
        
        //BB25 YACCA test legal predecessor is 24
        //BB25 RACFED setting adjustvalue successor 26
        
                        CONTROLLER_IN_HFP_FAILURE => {
        UpdSigEnd(24,25);
        
        UpdSigBegin(25);
        CheckSig(25);
                            /* Outport: '<Root>/Error' */
                            self.y.error = 2;
        UpdSigEnd(25,26);
        //BB26 YACCA test legal predecessor is 25
        //BB26 RACFED setting adjustvalue successor 27,28
        UpdSigBegin(26);     
        
                            if ((self.dw.temporal_counter_i1 >= 50) && (!self.y.hfp_do)) || (abs((3.0 - 0.5 * self.u.out_flow_cm3_s) - rtb_Diff) < 0.1) {
        CheckSig(26); 
        UpdSigEnd(26,27);
        
        //BB27 YACCA test legal predecessor is 26
        //BB27 RACFED setting adjustvalue successor 28
        UpdSigBegin(27);
        CheckSig(27); 
                                self.dw.is_monitor = CONTROLLER_IN_NO_ERR;
        AddRand(5);
                                /* Outport: '<Root>/Error' */
                                self.y.error = 0;
        UpdSigEnd(27,28);
                            } else { // for RACFED
        CheckSig(26); 
        UpdSigEnd(26,28);
                            }
        //BB28 YACCA test legal predecessor are 26, 27
        //BB28 RACFED setting adjustvalue successor 37
        UpdSigBegin(28);
        CheckSig(28);
        UpdSigEnd(28,37);
        
        //for BB37
        UpdSigBegin(37);
        CheckSig(37);
                        } //corresponds to the C break;
        
        //BB29 YACCA test legal predecessor 24
        //BB29 RACFED setting adjustvalue successor 30,31
        
                        CONTROLLER_IN_LFP_FAILURE => {
        
        UpdSigEnd(24,29);
        UpdSigBegin(29);  
                            /* Outport: '<Root>/Error' */
                            self.y.error = 1;
                            if ((self.dw.temporal_counter_i1 >= 50) && (!self.y.lfp_do)) || (abs((1.0 - 0.5 * self.u.out_flow_cm3_s) - rtb_Diff) < 0.1) {
        CheckSig(29);     
        UpdSigEnd(29,30);
        
        //BB30 YACCA test legal predecessor is 29
        //BB30 RACFED setting adjustvalue successor 31
        UpdSigBegin(30);
        CheckSig(30);
                                self.dw.is_monitor = CONTROLLER_IN_NO_ERR;
        
                                /* Outport: '<Root>/Error' */
                                self.y.error = 0;
        UpdSigEnd(30,31);
                            } else { // for RACFED
        CheckSig(29);      
        UpdSigEnd(29,31);
                            }
        
        //BB31   YACCA test legal predecessor are 29 & 30
        //BB31 RACFED setting adjustvalue successor 37
        
        UpdSigBegin(31);
        CheckSig(31);
        UpdSigEnd(31,37);
        
        //for BB37
        UpdSigBegin(37);
        CheckSig(37);
                        } //corresponds to the C break;
        
        //BB32 YACCA test legal predecessor is 24
        //BB32 RACFED setting adjustvalue successor 33,34,36
        //BB32 more than 2 instructions
        
                        _ => {
        UpdSigEnd(24,32); 
        UpdSigBegin(32); 
                            /* Outport: '<Root>/Error' */
                            /* case IN_NO_ERR: */
                            self.y.error = 0;
        
                            if self.y.lfp_do && (abs((1.0 - 0.5 * self.u.out_flow_cm3_s) - rtb_Diff) > 0.1) {
        CheckSig(32);
        UpdSigEnd(32,33); 
        
        //BB33  YACCA test legal predecessor is  32
        //BB33 RACFED setting adjustvalue successor 36
        //BB33 more than 2 instructions
        UpdSigBegin(33);
        CheckSig(33);
                                self.dw.is_monitor = CONTROLLER_IN_LFP_FAILURE;
        AddRand(87);    
                                self.dw.temporal_counter_i1 = 0;
        AddRand(88);  
                                /* Outport: '<Root>/Error' */
                                self.y.error = 1;
        AddRand(86);  
        
        UpdSigEnd(33,36);
        
        //BB34 YACCA test legal predecessor is 32
        //BB34 RACFED setting adjustvalue successor 35,36
                            } else if self.y.hfp_do && (abs((3.0 - 0.5 * self.u.out_flow_cm3_s) - rtb_Diff) > 0.1) {
        CheckSig(32);
        UpdSigEnd(32,35);
        
        //BB35 YACCA test legal predecessor is 34
        //BB35  RACFED setting adjustvalue successor 36
        //BB35 more than 2 instructions
        UpdSigBegin(35);
        CheckSig(35);
                                self.dw.is_monitor = CONTROLLER_IN_HFP_FAILURE;
        AddRand(44);    
                                self.dw.temporal_counter_i1 = 0;
        AddRand(45);     
                                /* Outport: '<Root>/Error' */
                                self.y.error = 2;
        AddRand(46);            
        
        UpdSigEnd(35,36); 
                            } else { //for RACFED
        CheckSig(32);
        UpdSigEnd(32,36);
                            }
        //BB 36
        //BB36 RACFED setting adjustvalue successor 37
        UpdSigBegin(36);
        CheckSig(36);
        UpdSigEnd(36,37);
        
        //for BB37
        UpdSigBegin(37);
        CheckSig(37);
        
                        } //Corresponds to the break of the C
        
                    } //End of the match
        
        //BB37   //YACCA test legal predecessor are 36 & 31 & 28 & 24
        //BB37  RACFED setting adjustvalue successor 38
        
        UpdSigEnd(37,38);
        
                } //end of the else
                /* End of Chart: '<Root>/Chart' */
        
                /* Update for UnitDelay: '<S2>/UD'
                *
                * Block description for '<S2>/UD':
                *
                *  Store in Global RAM
                */
        
        //BB38 YACCA test legal predecessor are 37 & 4
        //BB38 RACFED setting adjustvalue successor 0
        UpdSigBegin(38);
        CheckSig(38);
        
                self.dw.ud_dstate = rtb_TSamp;
        
        UpdSigEnd(38,0);
        
                unsafe { 
                /*CFCS.*/COUNT = /*CFCS.*/COUNT + 1 
                };
        
    }//end Function

    fn initialize(&mut self) {
        // No initialization code required
    }

    fn terminate(&mut self) {
        // No terminate code required
    }
}