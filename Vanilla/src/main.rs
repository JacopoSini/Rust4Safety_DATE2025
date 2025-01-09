#![no_std]
#![no_main]

use core::panic::PanicInfo;


// Variables just kept to not modify the GDB scripts to run the golden run for the vanilla version
#[no_mangle] //ensures that the name is kept exactly as it is written in the source code
pub static mut ERR_CODE : u64 = 0;
#[no_mangle]
pub static mut ID : u64 = 1 << 38;
#[no_mangle]
pub static mut last_predecessor_mask : u64 = 0;
#[no_mangle]
pub static mut terminated: u64 = 0;
#[no_mangle]
pub static mut Count : u64 = 0;
#[no_mangle]
pub static mut y : f64 = 0.0;


#[no_mangle]
pub extern "C" fn main() -> () {
    unsafe{ID = 0;}
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

        //Read outputs
        let hfp = controller.y.hfp_do;
        let lfp = controller.y.lfp_do;
        let error = controller.y.error;
        let status = controller.dw.is_controller;

        level_cm = level_cm + ((1.0 * if(controller.y.lfp_do) {1.0} else{0.0}) + (3.0 * if(controller.y.lfp_do) {1.0} else{0.0}) - (0.5 * out_flow_cm3_s))/10.0;

        unsafe{ /*CFCS.*/y = level_cm; }

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
//YACCA_TEST(1<<38 | 1<<0);

        let rtb_Diff : f64;
        let rtb_TSamp : f64;

        /* SampleTimeMath: '<S2>/TSamp' incorporates:
        *  Inport: '<Root>/Level_cm'
        *
        * About '<S2>/TSamp':
        *  y = u * K where K = 1 / ( w * Ts )
        */

        rtb_TSamp = self.u.level_cm * 10.0;

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

        /* Chart: '<Root>/Chart' incorporates:
        *  Inport: '<Root>/Level_cm'
        *  Inport: '<Root>/Out_Flow_cm3_s'
        *  Outport: '<Root>/Error'
        *  Outport: '<Root>/HFP_DO'
        *  Outport: '<Root>/LFP_DO'
        */

    
        if(self.dw.temporal_counter_i1 < 63){

            self.dw.temporal_counter_i1 += 1;

        } 

        if self.dw.is_active_c3_controller == 0 {


            self.dw.is_active_c3_controller = 1;
            self.dw.is_controller = CONTROLLER_IN_OFF;

            /* Outport: '<Root>/HFP_DO' */
            self.y.hfp_do = false;

            /* Outport: '<Root>/LFP_DO' */
            self.y.lfp_do = false;
            self.dw.is_monitor = CONTROLLER_IN_NO_ERR;

            /* Outport: '<Root>/Error' */
            self.y.error = 0;


        } else {

            match self.dw.is_controller {


                CONTROLLER_IN_HFP_ON => {

                    if ((rtb_Diff > 0.0) && (self.u.level_cm > 155.0)) || (self.y.error == 2) {

                        self.dw.is_controller = CONTROLLER_IN_LFP_ON;


                    } else if self.u.level_cm > 180.0 {

                        self.dw.is_controller = CONTROLLER_IN_OFF;
                        
                        /* Outport: '<Root>/HFP_DO' */
                        self.y.hfp_do = false;

                        /* Outport: '<Root>/LFP_DO' */
                        self.y.lfp_do = false;


                    } else {

                        /* Outport: '<Root>/HFP_DO' */
                        self.y.hfp_do = true;

                        /* Outport: '<Root>/LFP_DO' */
                        self.y.lfp_do = false;

                    }


                }


                CONTROLLER_IN_LFP_ON => {


                    if (rtb_Diff > 0.0) && (self.u.level_cm > 180.0) {

                        self.dw.is_controller = CONTROLLER_IN_OFF;

                        /* Outport: '<Root>/HFP_DO' */
                        self.y.hfp_do = false;

                        /* Outport: '<Root>/LFP_DO' */
                        self.y.lfp_do = false;

                    } else if ((rtb_Diff < 0.0) && (self.u.level_cm < 150.0) && 
                                    (self.y.error != 2)) || (self.y.error == 1) {

                        self.dw.is_controller = CONTROLLER_IN_HFP_ON;


                    } else {

                        /* Outport: '<Root>/HFP_DO' */
                        self.y.hfp_do = false;

                        /* Outport: '<Root>/LFP_DO' */
                        self.y.lfp_do = true;

                    }


                }

                _ => {

                    /* case IN_OFF: */
                    if (rtb_Diff < 0.0) && (self.u.level_cm < 150.0) {

                        self.dw.is_controller = CONTROLLER_IN_HFP_ON;

                    } else if (rtb_Diff < 0.0) && (self.u.level_cm < 175.0) {



                        self.dw.is_controller = CONTROLLER_IN_LFP_ON;
                  } 

                }

            }


            match self.dw.is_monitor {


                CONTROLLER_IN_HFP_FAILURE => {

                    /* Outport: '<Root>/Error' */
                    self.y.error = 2;


                    if ((self.dw.temporal_counter_i1 >= 50) && (!self.y.hfp_do)) || (abs((3.0 - 0.5 * self.u.out_flow_cm3_s) - rtb_Diff) < 0.1) {

                        self.dw.is_monitor = CONTROLLER_IN_NO_ERR;

                        /* Outport: '<Root>/Error' */
                        self.y.error = 0;

                    } 

                }


                CONTROLLER_IN_LFP_FAILURE => {

                    /* Outport: '<Root>/Error' */
                    self.y.error = 1;
                    if ((self.dw.temporal_counter_i1 >= 50) && (!self.y.lfp_do)) || (abs((1.0 - 0.5 * self.u.out_flow_cm3_s) - rtb_Diff) < 0.1) {

                        self.dw.is_monitor = CONTROLLER_IN_NO_ERR;
                        
                        /* Outport: '<Root>/Error' */
                        self.y.error = 0;

                    } 

                }

                _ => {

                    /* Outport: '<Root>/Error' */
                    /* case IN_NO_ERR: */
                    self.y.error = 0;

                    if self.y.lfp_do && (abs((1.0 - 0.5 * self.u.out_flow_cm3_s) - rtb_Diff) > 0.1) {

                        self.dw.is_monitor = CONTROLLER_IN_LFP_FAILURE;
                        self.dw.temporal_counter_i1 = 0;

                        /* Outport: '<Root>/Error' */
                        self.y.error = 1;

                    } else if self.y.hfp_do && (abs((3.0 - 0.5 * self.u.out_flow_cm3_s) - rtb_Diff) > 0.1) {

                        self.dw.is_monitor = CONTROLLER_IN_HFP_FAILURE;
                        self.dw.temporal_counter_i1 = 0;

                        /* Outport: '<Root>/Error' */
                        self.y.error = 2;

                    } 


                }
               
            }

        } //end of the else
        /* End of Chart: '<Root>/Chart' */

        /* Update for UnitDelay: '<S2>/UD'
        *
        * Block description for '<S2>/UD':
        *
        *  Store in Global RAM
        */



        self.dw.ud_dstate = rtb_TSamp;


        unsafe { 
            /*CFCS.*/Count = /*CFCS.*/Count + 1 
        };
    }//end Function

    fn initialize(&mut self) {
        // No initialization code required
    }

    fn terminate(&mut self) {
        // No terminate code required
    }
}