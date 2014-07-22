#ifndef RS274NGC_INTERP_H
#define RS274NGC_INTERP_H
#include "rs274ngc.h"

class Interp
{

 public:
   Interp();
   ~Interp();

/* Interface functions to call to tell the interpreter what to do.
   Return values indicate status of execution.
   These functions may change the state of the interpreter. */

// close the currently open NC code file
   int close();

// execute a line of NC code
#ifndef NOT_OLD_EMC_INTERP_COMPATIBLE
   int execute(const char *command = 0);
#else
   int execute();
#endif

   int execute(const char *command, int line_no);       //used for MDI calls to specify the pseudo MDI line number

// stop running
   int exit();

// get ready to run
   int init();

// load a tool table
   int load_tool_table();

// open a file of NC code
   int open(const char *filename);

// read the mdi or the next line of the open NC code file
   int read(const char *mdi = 0);

// reset yourself
   int reset();

// restore interpreter variables from a file
   int restore_parameters(const char *filename);

// save interpreter variables to file
   int save_parameters(const char *filename, const double parameters[]);

// synchronize your internal model with the external world
   int synch();

/* Interface functions to call to get information from the interpreter.
   If a function has a return value, the return value contains the information.
   If a function returns nothing, information is copied into one of the
   arguments to the function. These functions do not change the state of
   the interpreter. */

// copy active G codes into array [0]..[15]
   void active_g_codes(int *codes);

// copy active M codes into array [0]..[9]
   void active_m_codes(int *codes);

// copy active F, S settings into array [0]..[2]
   void active_settings(double *settings);

// copy the text of the error message whose number is error_code into the
// error_text array, but stop at max_size if the text is longer.
   void error_text(int error_code, char *error_text, int max_size);

   void setError(const char *fmt, ...);

// copy the name of the currently open file into the file_name array,
// but stop at max_size if the name is longer
   void file_name(char *file_name, int max_size);

// return the length of the most recently read line
   int line_length();

// copy the text of the most recently read line into the line_text array,
// but stop at max_size if the text is longer
   void line_text(char *line_text, int max_size);

// return the current sequence number (how many lines read)
   int sequence_number();

// copy the function name from the stack_index'th position of the
// function call stack at the time of the most recent error into
// the function name string, but stop at max_size if the name is longer
   void stack_name(int stack_index, char *function_name, int max_size);

// Get the parameter file name from the ini file.
   int ini_load(const char *filename);

   int line()
   {
      return sequence_number();
   }

   char *command(char *buf, int len)
   {
      line_text(buf, len);
      return buf;
   }

   char *file(char *buf, int len)
   {
      file_name(buf, len);
      return buf;
   }

   int init_tool_parameters();
   int default_tool_parameters();
   int set_tool_parameters();

 private:

/* Function prototypes for all  functions */

   int arc_data_comp_ijk(int move, int plane, int side, double tool_radius,
                         double current_x, double current_y,
                         double end_x, double end_y,
                         int ij_absolute, double i_number, double j_number, double *center_x, double *center_y, int *turn, double tolerance);
   int arc_data_comp_r(int move, int plane, int side, double tool_radius,
                       double current_x, double current_y, double end_x,
                       double end_y, double big_radius, double *center_x, double *center_y, int *turn, double tolerance);
   int arc_data_ijk(int move, int plane, double current_x, double current_y,
                    double end_x, double end_y,
                    int ij_absolute, double i_number, double j_number, double *center_x, double *center_y, int *turn, double tolerance);
   int arc_data_r(int move, int plane, double current_x, double current_y,
                  double end_x, double end_y, double radius, double *center_x, double *center_y, int *turn, double tolerance);
   int check_g_codes(block_pointer block, setup_pointer settings);
   int check_items(block_pointer block, setup_pointer settings);
   int check_m_codes(block_pointer block);
   int check_other_codes(block_pointer block);
   int close_and_downcase(char *line);
   int convert_nurbs(int move, block_pointer block, setup_pointer settings);
   int convert_spline(int move, block_pointer block, setup_pointer settings);
   int comp_get_current(setup_pointer settings, double *x, double *y, double *z);
   int comp_set_current(setup_pointer settings, double x, double y, double z);
   int comp_get_programmed(setup_pointer settings, double *x, double *y, double *z);
   int comp_set_programmed(setup_pointer settings, double x, double y, double z);
   int convert_arc(int move, block_pointer block, setup_pointer settings);
   int convert_arc2(int move, block_pointer block,
                    setup_pointer settings,
                    double *current1, double *current2, double *current3,
                    double end1, double end2, double end3,
                    double AA_end, double BB_end, double CC_end, double u_end, double v_end, double w_end, double offset1, double offset2);

   int convert_arc_comp1(int move, block_pointer block,
                         setup_pointer settings,
                         double end_x, double end_y, double end_z,
                         double offset_x, double offset_y, double AA_end, double BB_end, double CC_end, double u_end, double v_end, double w_end);

   int convert_arc_comp2(int move, block_pointer block,
                         setup_pointer settings,
                         double end_x, double end_y, double end_z,
                         double offset_x, double offset_y, double AA_end, double BB_end, double CC_end, double u_end, double v_end, double w_end);
   char arc_axis1(int plane);
   char arc_axis2(int plane);
   int convert_axis_offsets(int g_code, block_pointer block, setup_pointer settings);
   int convert_param_comment(char *comment, char *expanded, int len);
   int convert_comment(char *comment);
   int convert_control_mode(int g_code, double tolerance, double naivecam_tolerance, setup_pointer settings);
   int convert_adaptive_mode(int g_code, setup_pointer settings);

   int convert_coordinate_system(int g_code, setup_pointer settings);
   int convert_cutter_compensation(int g_code, block_pointer block, setup_pointer settings);
   int convert_cutter_compensation_off(setup_pointer settings);
   int convert_cutter_compensation_on(int side, block_pointer block, setup_pointer settings);
   int convert_cycle(int motion, block_pointer block, setup_pointer settings);
   int convert_cycle_g81(block_pointer block, CANON_PLANE plane, double x, double y, double clear_z, double bottom_z);
   int convert_cycle_g82(block_pointer block, CANON_PLANE plane, double x, double y, double clear_z, double bottom_z, double dwell);
   int convert_cycle_g73(block_pointer block, CANON_PLANE plane, double x, double y, double r, double clear_z, double bottom_z, double delta);
   int convert_cycle_g83(block_pointer block, CANON_PLANE plane, double x, double y, double r, double clear_z, double bottom_z, double delta);
   int convert_cycle_g84(block_pointer block, CANON_PLANE plane, double x, double y,
                         double clear_z, double bottom_z, CANON_DIRECTION direction, CANON_SPEED_FEED_MODE mode);
   int convert_cycle_g85(block_pointer block, CANON_PLANE plane, double x, double y, double clear_z, double bottom_z);
   int convert_cycle_g86(block_pointer block, CANON_PLANE plane, double x, double y, double clear_z, double bottom_z, double dwell, CANON_DIRECTION direction);
   int convert_cycle_g87(block_pointer block, CANON_PLANE plane, double x, double offset_x,
                         double y, double offset_y, double r, double clear_z, double middle_z, double bottom_z, CANON_DIRECTION direction);
   int convert_cycle_g88(block_pointer block, CANON_PLANE plane, double x, double y, double bottom_z, double dwell, CANON_DIRECTION direction);
   int convert_cycle_g89(block_pointer block, CANON_PLANE plane, double x, double y, double clear_z, double bottom_z, double dwell);
   int convert_cycle_xy(int motion, block_pointer block, setup_pointer settings);
   int convert_cycle_yz(int motion, block_pointer block, setup_pointer settings);
   int convert_cycle_zx(int motion, block_pointer block, setup_pointer settings);
   int convert_cycle_uv(int motion, block_pointer block, setup_pointer settings);
   int convert_cycle_vw(int motion, block_pointer block, setup_pointer settings);
   int convert_cycle_wu(int motion, block_pointer block, setup_pointer settings);
   int convert_distance_mode(int g_code, setup_pointer settings);
   int convert_ijk_distance_mode(int g_code, setup_pointer settings);
   int convert_lathe_diameter_mode(int g_code, block_pointer block, setup_pointer settings);
   int convert_dwell(setup_pointer settings, double time);
   int convert_feed_mode(int g_code, setup_pointer settings);
   int convert_feed_rate(block_pointer block, setup_pointer settings);
   int convert_g(block_pointer block, setup_pointer settings);
   int convert_home(int move, block_pointer block, setup_pointer settings);
   int convert_savehome(int move, block_pointer block, setup_pointer settings);
   int convert_length_units(int g_code, setup_pointer settings);
   int convert_m(block_pointer block, setup_pointer settings);
   int convert_modal_0(int code, block_pointer block, setup_pointer settings);
   int convert_motion(int motion, block_pointer block, setup_pointer settings);
   int convert_probe(block_pointer block, int g_code, setup_pointer settings);
   int convert_retract_mode(int g_code, setup_pointer settings);
   int convert_setup(block_pointer block, setup_pointer settings);
   int convert_setup_tool(block_pointer block, setup_pointer settings);
   int convert_set_plane(int g_code, setup_pointer settings);
   int convert_speed(block_pointer block, setup_pointer settings);
   int convert_spindle_mode(block_pointer block, setup_pointer settings);
   int convert_stop(block_pointer block, setup_pointer settings);
   int convert_straight(int move, block_pointer block, setup_pointer settings);
   int convert_straight_comp1(int move, block_pointer block,
                              setup_pointer settings,
                              double px, double py, double end_z, double AA_end, double BB_end, double CC_end, double u_end, double v_end, double w_end);
   int convert_straight_comp2(int move, block_pointer block,
                              setup_pointer settings,
                              double px, double py, double end_z, double AA_end, double BB_end, double CC_end, double u_end, double v_end, double w_end);
   int convert_threading_cycle(block_pointer block, setup_pointer settings, double end_x, double end_y, double end_z);
   int convert_tool_change(setup_pointer settings);
   int convert_tool_length_offset(int g_code, block_pointer block, setup_pointer settings);
   int convert_tool_select(block_pointer block, setup_pointer settings);
   int cycle_feed(block_pointer block, CANON_PLANE plane, double end1, double end2, double end3);
   int cycle_traverse(block_pointer block, CANON_PLANE plane, double end1, double end2, double end3);
   int enhance_block(block_pointer block, setup_pointer settings);
   int execute_binary(double *left, int operation, double *right);
   int execute_binary1(double *left, int operation, double *right);
   int execute_binary2(double *left, int operation, double *right);
   int execute_block(block_pointer block, setup_pointer settings);
   int execute_unary(double *double_ptr, int operation);
   double find_arc_length(double x1, double y1, double z1, double center_x, double center_y, int turn, double x2, double y2, double z2);
   int find_current_in_system(setup_pointer s, int system, double *x, double *y, double *z, double *a, double *b, double *c, double *u, double *v, double *w);
   int find_ends(block_pointer block, setup_pointer settings,
                 double *px, double *py, double *pz, double *AA_p, double *BB_p, double *CC_p, double *u_p, double *v_p, double *w_p);
   int find_relative(double x1, double y1, double z1,
                     double AA_1, double BB_1, double CC_1,
                     double u_1, double v_1, double w_1,
                     double *x2, double *y2, double *z2,
                     double *AA_2, double *BB_2, double *CC_2, double *u_2, double *v_2, double *w_2, setup_pointer settings);
   double find_straight_length(double x2, double y2, double z2,
                               double AA_2, double BB_2, double CC_2,
                               double u_w, double v_2, double w_2,
                               double x1, double y1, double z1, double AA_1, double BB_1, double CC_1, double u_1, double v_1, double w_1);
   int find_tool_pocket(setup_pointer settings, int toolno, int *pocket);
   double find_turn(double x1, double y1, double center_x, double center_y, int turn, double x2, double y2);
   int init_block(block_pointer block);
   int inverse_time_rate_arc(double x1, double y1, double z1,
                             double cx, double cy, int turn, double x2, double y2, double z2, block_pointer block, setup_pointer settings);
   int inverse_time_rate_straight(double end_x, double end_y, double end_z,
                                  double AA_end, double BB_end, double CC_end,
                                  double u_end, double v_end, double w_end, block_pointer block, setup_pointer settings);
   int move_endpoint_and_flush(setup_pointer, double, double);
   int parse_line(char *line, block_pointer block, setup_pointer settings);
   int precedence(int an_operator);
   int read_a(char *line, int *counter, block_pointer block, double *parameters);
   int read_atan(char *line, int *counter, double *double_ptr, double *parameters);
   int read_atsign(char *line, int *counter, block_pointer block, double *parameters);
   int read_b(char *line, int *counter, block_pointer block, double *parameters);
   int read_c(char *line, int *counter, block_pointer block, double *parameters);
   int read_carat(char *line, int *counter, block_pointer block, double *parameters);
   int read_comment(char *line, int *counter, block_pointer block, double *parameters);
   int read_semicolon(char *line, int *counter, block_pointer block, double *parameters);
   int read_d(char *line, int *counter, block_pointer block, double *parameters);
   int read_e(char *line, int *counter, block_pointer block, double *parameters);
   int read_f(char *line, int *counter, block_pointer block, double *parameters);
   int read_g(char *line, int *counter, block_pointer block, double *parameters);
   int read_h(char *line, int *counter, block_pointer block, double *parameters);
   int read_i(char *line, int *counter, block_pointer block, double *parameters);
   int read_integer_unsigned(char *line, int *counter, int *integer_ptr);
   int read_integer_value(char *line, int *counter, int *integer_ptr, double *parameters);
   int read_items(block_pointer block, char *line, double *parameters);
   int read_j(char *line, int *counter, block_pointer block, double *parameters);
   int read_k(char *line, int *counter, block_pointer block, double *parameters);
   int read_l(char *line, int *counter, block_pointer block, double *parameters);
   int read_n_number(char *line, int *counter, block_pointer block);
   int read_m(char *line, int *counter, block_pointer block, double *parameters);
   int read_o(char *line, int *counter, block_pointer block, double *parameters);
   int free_named_parameters(int level, setup_pointer settings);
   int read_one_item(char *line, int *counter, block_pointer block, double *parameters);
   int read_operation(char *line, int *counter, int *operation);
   int read_operation_unary(char *line, int *counter, int *operation);
   int read_p(char *line, int *counter, block_pointer block, double *parameters);
   int store_named_param(char *nameBuf, double value);
   int add_named_param(char *nameBuf);
   int find_named_param(char *nameBuf, int *status, double *value);
   int read_name(char *line, int *counter, char *nameBuf);
   int read_named_parameter(char *line, int *counter, double *double_ptr, double *parameters);
   int read_parameter(char *line, int *counter, double *double_ptr, double *parameters);
   int read_parameter_setting(char *line, int *counter, block_pointer block, double *parameters);
   int read_named_parameter_setting(char *line, int *counter, char **param, double *parameters);
   int read_q(char *line, int *counter, block_pointer block, double *parameters);
   int read_r(char *line, int *counter, block_pointer block, double *parameters);
   int read_real_expression(char *line, int *counter, double *hold2, double *parameters);
   int read_real_number(char *line, int *counter, double *double_ptr);
   int read_real_value(char *line, int *counter, double *double_ptr, double *parameters);
   int read_s(char *line, int *counter, block_pointer block, double *parameters);
   int read_t(char *line, int *counter, block_pointer block, double *parameters);
   int read_text(const char *command, FILE * inport, char *raw_line, char *line, int *length);
   int read_unary(char *line, int *counter, double *double_ptr, double *parameters);
   int read_u(char *line, int *counter, block_pointer block, double *parameters);
   int read_v(char *line, int *counter, block_pointer block, double *parameters);
   int read_w(char *line, int *counter, block_pointer block, double *parameters);
   int read_x(char *line, int *counter, block_pointer block, double *parameters);
   int read_y(char *line, int *counter, block_pointer block, double *parameters);
   int read_z(char *line, int *counter, block_pointer block, double *parameters);
   int refresh_actual_position(setup_pointer settings);
   void rotate(double *x, double *y, double t);
   int set_probe_data(setup_pointer settings);
   int write_g_codes(block_pointer block, setup_pointer settings);
   int write_m_codes(block_pointer block, setup_pointer settings);
   int write_settings(setup_pointer settings);
   int unwrap_rotary(double *, double, double, double, char);
   bool isreadonly(int index);

   // O_word stuff

   int findFile(                // ARGUMENTS
                  char *direct, // the directory to start looking in
                  char *target, // the name of the file to find
                  char *foundFileDirect);       // where to store the result

   int control_save_offset(     /* ARGUMENTS                   */
                             int line,  /* (o-word) line number        */
                             block_pointer block,       /* pointer to a block of RS274/NGC instructions */
                             setup_pointer settings);   /* pointer to machine settings */

   int control_find_oword(      /* ARGUMENTS                   */
                            block_pointer block,        /* block pointer to get (o-word) name        */
                            setup_pointer settings,     /* pointer to machine settings */
                            int *o_index);      /* the index of o-word (returned) */

   int control_back_to(         /* ARGUMENTS                   */
                         block_pointer block,   // pointer to block
                         setup_pointer settings);       /* pointer to machine settings */

   int convert_control_functions(       /* ARGUMENTS           */
                                   block_pointer block, /* pointer to a block of RS274/NGC instructions */
                                   setup_pointer settings);     /* pointer to machine settings */

   void doLog(char *fmt, ...);

   FILE *log_file;

/* Internal arrays */
   static const int _gees[];
   static const int _ems[];
   static const int _required_parameters[];
   static const int _readonly_parameters[];
   static const int _n_readonly_parameters;
   read_function_pointer _readers[256];
   static const read_function_pointer default_readers[256];

   static setup _setup;

   enum
   {
      AXIS_MASK_X = 1, AXIS_MASK_Y = 2, AXIS_MASK_Z = 4,
      AXIS_MASK_A = 8, AXIS_MASK_B = 16, AXIS_MASK_C = 32,
      AXIS_MASK_U = 64, AXIS_MASK_V = 128, AXIS_MASK_W = 256,
   };
};

#endif
