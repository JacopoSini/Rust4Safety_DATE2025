#![no_std]
#![no_main]

use core::panic::PanicInfo;


#[no_mangle] //ensures that the name is kept exactly as it is written in the source code
pub static mut ERR_CODE : u64 = 0;
#[no_mangle]
pub static mut ID : u64 = 1 << 38;
#[no_mangle]
pub static mut last_predecessor_mask : u64 = 0;
#[no_mangle]
pub static mut Count : u64 = 0;
#[no_mangle]
pub static mut terminated: u64 = 0;
#[no_mangle]
pub static mut y : f64 = 0.0;


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

        //Read outputs
        let hfp = controller.y.hfp_do;
        let lfp = controller.y.lfp_do;
        let error = controller.y.error;
        let status = controller.dw.is_controller;

        level_cm = level_cm + ((1.0 * if(controller.y.lfp_do) {1.0} else{0.0}) + (3.0 * if(controller.y.lfp_do) {1.0} else{0.0}) - (0.5 * out_flow_cm3_s))/10.0;

        unsafe{ /*CFCS.*/y = level_cm; }
        //Only when run on PCs
        let ERR_CODE_local = unsafe {/*CFCS.*/ERR_CODE};
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

fn YACCA_TEST(predecessors_mask : u64){
    unsafe{
        /*CFCS.*/last_predecessor_mask = predecessors_mask;
        /*CFCS.*/ERR_CODE = /*CFCS.*/ERR_CODE + if /*CFCS.*/ID & (! predecessors_mask ) > 0 {1} else {0};
    }
    //Only when run on PCs
    let ERR_CODE_local = unsafe{/*CFCS.*/ERR_CODE};
    let ID_local = unsafe{/*CFCS.*/ID};
    if(ERR_CODE_local > 0){
        ;//println!("Failed YACCA TEST for mask {predecessors_mask} when ID is {ID_local}");
    }
    /* // Not to be used in normal operations
    if ERR_CODE != 0{
        ERR_CODE = 0;
    }
    */
}

fn YACCA_SET(BB_ID : u8){
    if (BB_ID < 63){
        unsafe{
            /*CFCS.*/ID = (/*CFCS.*/ID & (! /*CFCS.*/last_predecessor_mask)) ^ (1<<BB_ID);
        }
    }
    else {
        unsafe{
            /*CFCS.*/ERR_CODE = 255;
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
YACCA_TEST(1<<38 | 1<<0);

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

YACCA_SET(0);
        
//BB1 YACCA test legal predecessor is 0
YACCA_TEST(1<<0);        
        if(self.dw.temporal_counter_i1 < 63){
YACCA_SET(1);

//BB2 YACCA test legal predecessor is 1
YACCA_TEST(1<<1);
            self.dw.temporal_counter_i1 += 1;
YACCA_SET(2);
        } else { //YACCA
YACCA_SET(1);
        }
    

//BB3 YACCA test legal predecessor are 2 or 1

YACCA_TEST(1<<2 | 1<<1);

        if self.dw.is_active_c3_controller == 0 {
YACCA_SET(3);

//BB4 YACCA test legal predecessor is 3

YACCA_TEST(1 << 3);

            self.dw.is_active_c3_controller = 1;
            self.dw.is_controller = CONTROLLER_IN_OFF;

            /* Outport: '<Root>/HFP_DO' */
            self.y.hfp_do = false;

            /* Outport: '<Root>/LFP_DO' */
            self.y.lfp_do = false;
            self.dw.is_monitor = CONTROLLER_IN_NO_ERR;

            /* Outport: '<Root>/Error' */
            self.y.error = 0;

YACCA_SET(4);
//BB5 YACCA test legal predecessor is 3

        } else {

YACCA_SET(3);

YACCA_TEST(1 << 3);

YACCA_SET(5);//add by Maz in the C source code, not added during our meeting

//BB6 YACCA test legal predecessor is 5

YACCA_TEST(1 << 5);    
            match self.dw.is_controller {
//BB7 YACCA test legal predecessor is 6 (in all "cases")

                CONTROLLER_IN_HFP_ON => {
YACCA_SET(6);

YACCA_TEST(1 << 6);

YACCA_SET(7);

//BB8   YACCA test legal predecessor is 7
YACCA_TEST(1 << 7);


                    if ((rtb_Diff > 0.0) && (self.u.level_cm > 155.0)) || (self.y.error == 2) {
YACCA_SET(8);

//BB9   YACCA test legal predecessor is 8

YACCA_TEST(1 << 8);
                        self.dw.is_controller = CONTROLLER_IN_LFP_ON;
YACCA_SET(9);

//BB10 YACCA test legal predecessor is 9

                    } else if self.u.level_cm > 180.0 {
YACCA_SET(8);
YACCA_TEST(1 << 8);
                        self.dw.is_controller = CONTROLLER_IN_OFF;
                        
                        /* Outport: '<Root>/HFP_DO' */
                        self.y.hfp_do = false;

                        /* Outport: '<Root>/LFP_DO' */
                        self.y.lfp_do = false;
YACCA_SET(10);

//BB11 YACCA test legal predecessor is 7

                    } else {
YACCA_SET(8);
YACCA_TEST(1 << 8);
                        /* Outport: '<Root>/HFP_DO' */
                        self.y.hfp_do = true;

                        /* Outport: '<Root>/LFP_DO' */
                        self.y.lfp_do = false;
YACCA_SET(11);
                    }

//BB12 YACCA test legal predecessor are 11,10,8,9

YACCA_TEST((1 << 8) | (1 << 9) | (1 << 10) | (1 << 11));
YACCA_SET(12);
YACCA_TEST(1 << 12); //for the BB24
                }

//BB13 YACCA test legal predecessor 6
                CONTROLLER_IN_LFP_ON => {
YACCA_SET(6);

YACCA_TEST(1 << 6);

                    if (rtb_Diff > 0.0) && (self.u.level_cm > 180.0) {
YACCA_SET(13);

//BB14 YACCA test legal predecessor 13

YACCA_TEST(1 << 13);
                        self.dw.is_controller = CONTROLLER_IN_OFF;

                        /* Outport: '<Root>/HFP_DO' */
                        self.y.hfp_do = false;

                        /* Outport: '<Root>/LFP_DO' */
                        self.y.lfp_do = false;
YACCA_SET(14);

//BB15 YACCA test legal predecessor is 13
YACCA_TEST(1 << 14);
                    } else if ((rtb_Diff < 0.0) && (self.u.level_cm < 150.0) && 
                                    (self.y.error != 2)) || (self.y.error == 1) {
YACCA_SET(13);

YACCA_TEST(1 << 13);
                        
YACCA_SET(15);

//BB16 YACCA test legal predecessor is 15
YACCA_TEST(1 << 15);
                        self.dw.is_controller = CONTROLLER_IN_HFP_ON;
YACCA_SET(16);


//BB17 YACCA test legal predecessor is 13

                    } else {
YACCA_SET(13);
YACCA_TEST(1 << 13);

                        /* Outport: '<Root>/HFP_DO' */
                        self.y.hfp_do = false;

                        /* Outport: '<Root>/LFP_DO' */
                        self.y.lfp_do = true;
YACCA_SET(17);
                    }

//BB18 YACCA test legal predecessor are 17 & 16 & 14 & 13 

YACCA_TEST((1 << 14) | (1 << 15) | (1 << 16) | (1 << 17));
YACCA_SET(18);
YACCA_TEST(1 << 18); //For BB24
                }


//BB19 YACCA test legal predecessor is 6
                _ => {
YACCA_SET(6);
YACCA_TEST(1 << 6);
                    /* case IN_OFF: */
                    if (rtb_Diff < 0.0) && (self.u.level_cm < 150.0) {
YACCA_SET(19);

//BB20 YACCA test legal predecessor is 19

YACCA_TEST(1 << 19);
                        self.dw.is_controller = CONTROLLER_IN_HFP_ON;
YACCA_SET(20);

//BB21 YACCA test legal predecessor is 19

                    } else if (rtb_Diff < 0.0) && (self.u.level_cm < 175.0) {
YACCA_SET(19);

YACCA_TEST(1 << 19);
YACCA_SET(21);
//BB22 YACCA test legal predecessor is 21

YACCA_TEST(1 << 21);
                        self.dw.is_controller = CONTROLLER_IN_LFP_ON;
YACCA_SET(22);
                    } else { //YACCA
YACCA_SET(19);
                    }

//BB23 YACCA test legal predecessor are 22 & 21 & 20 & 19       

YACCA_TEST((1 << 19) | (1 << 20) | (1 << 21) | (1 << 22));
YACCA_SET(23); //SET_OTHER_DOMAIN????

YACCA_TEST(1 << 23); //for BB24
                }
//BB24
//TEST(1<<11 | 1<<16 | 1<<20) 
            }

//SET(24u);



            match self.dw.is_monitor {

//BB25 YACCA test legal predecessor is 24

                CONTROLLER_IN_HFP_FAILURE => {
YACCA_SET(24);

YACCA_TEST(1<< 24);

                    /* Outport: '<Root>/Error' */
                    self.y.error = 2;
YACCA_SET(25);

//BB26 YACCA test legal predecessor is 25
YACCA_TEST(1 << 25);

                    if ((self.dw.temporal_counter_i1 >= 50) && (!self.y.hfp_do)) || (abs((3.0 - 0.5 * self.u.out_flow_cm3_s) - rtb_Diff) < 0.1) {
YACCA_SET(26);
//BB27 YACCA test legal predecessor is 26 

YACCA_TEST(1 << 26);
                        self.dw.is_monitor = CONTROLLER_IN_NO_ERR;

                        /* Outport: '<Root>/Error' */
                        self.y.error = 0;
YACCA_SET(27);
                    } else { // for YACCA
YACCA_SET(26);
                    }
//BB28 YACCA test legal predecessor are 26, 27

YACCA_TEST((1 << 26) | (1 << 27));
YACCA_SET(28);
YACCA_TEST(1 << 28); //for BB Y
                }

//BB29 YACCA test legal predecessor 24
                CONTROLLER_IN_LFP_FAILURE => {
YACCA_SET(24);
YACCA_TEST(1 << 24);
                    /* Outport: '<Root>/Error' */
                    self.y.error = 1;
                    if ((self.dw.temporal_counter_i1 >= 50) && (!self.y.lfp_do)) || (abs((1.0 - 0.5 * self.u.out_flow_cm3_s) - rtb_Diff) < 0.1) {
YACCA_SET(29);

//BB30 YACCA test legal predecessor is 29

YACCA_TEST(1 << 29);
                        self.dw.is_monitor = CONTROLLER_IN_NO_ERR;
                        
                        /* Outport: '<Root>/Error' */
                        self.y.error = 0;
YACCA_SET(30);
                    } else { // for YACCA
YACCA_SET(29);
                    }

//BB31 YACCA test legal predecessor are 29 & 30

YACCA_TEST((1 << 29) | (1 << 30));
YACCA_SET(31);
YACCA_TEST(1 << 31); //for BB36
                }

//BB32 YACCA test legal predecessor is 24

                _ => {
YACCA_SET(24);
YACCA_TEST(1 << 24);
                    /* Outport: '<Root>/Error' */
                    /* case IN_NO_ERR: */
                    self.y.error = 0;

                    if self.y.lfp_do && (abs((1.0 - 0.5 * self.u.out_flow_cm3_s) - rtb_Diff) > 0.1) {
YACCA_SET(32);

//BB33  YACCA test legal predecessor is  32

YACCA_TEST(1 << 32);
                        self.dw.is_monitor = CONTROLLER_IN_LFP_FAILURE;
                        self.dw.temporal_counter_i1 = 0;

                        /* Outport: '<Root>/Error' */
                        self.y.error = 1;
YACCA_SET(33);
//BB34 YACCA test legal predecessor is 32
                    } else if self.y.hfp_do && (abs((3.0 - 0.5 * self.u.out_flow_cm3_s) - rtb_Diff) > 0.1) {
YACCA_SET(32);
YACCA_TEST(1 << 32);

YACCA_SET(34);
//BB35 YACCA test legal predecessor is 34  

YACCA_TEST(1<<34);
                        self.dw.is_monitor = CONTROLLER_IN_HFP_FAILURE;
                        self.dw.temporal_counter_i1 = 0;

                        /* Outport: '<Root>/Error' */
                        self.y.error = 2;
YACCA_SET(35);
                    } else { //for YACCA
YACCA_SET(32);
                    }
YACCA_TEST((1 << 32) | (1 << 33) | (1 << 34) | (1 << 35));

YACCA_SET(36);
YACCA_TEST(1 << 36); //for BB36

                }
               
            }

//BB37   //YACCA test legal predecessor are 36 & 31 & 28 & 24



YACCA_TEST((1 << 36) | (1 << 31) | (1 << 28) | (1 << 24));

YACCA_SET(37);
        } //end of the else
        /* End of Chart: '<Root>/Chart' */

        /* Update for UnitDelay: '<S2>/UD'
        *
        * Block description for '<S2>/UD':
        *
        *  Store in Global RAM
        */

//BB38 YACCA test legal predecessor are 37 & 4
YACCA_TEST((1 << 37) | (1 << 4));

        self.dw.ud_dstate = rtb_TSamp;

YACCA_SET(38);

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