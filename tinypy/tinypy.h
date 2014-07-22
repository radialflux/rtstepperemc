#ifndef _TINYPY_H
#define _TINYPY_H

#include <setjmp.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <pthread.h>

#if (defined(__WIN32__) || defined(_WINDOWS))
   #define DLL_EXPORT __declspec(dllexport)
#else
   #define DLL_EXPORT __attribute__ ((visibility("default")))
#endif

#ifdef __GNUC__
#define tp_inline __inline__
#endif

#ifdef _MSC_VER
#define tp_inline __inline
#endif

#ifndef tp_inline
#error "Unsuported compiler"
#endif

#define tp_malloc(x) calloc((x),1)
#define tp_realloc(x,y) realloc(x,y)
#define tp_free(x) free(x)

enum TP_OBJ_TYPE {
    TP_NONE,TP_NUMBER,TP_STRING,TP_DICT,
    TP_LIST,TP_FNC,TP_DATA,
};

typedef double tp_num;

typedef struct tp_number_ {
    enum TP_OBJ_TYPE type;
    tp_num val;
} tp_number_;
typedef struct tp_string_ {
    enum TP_OBJ_TYPE type;
    struct _tp_string *info;
    char const *val;
    int len;
} tp_string_;
typedef struct tp_list_ {
    enum TP_OBJ_TYPE type;
    struct _tp_list *val;
} tp_list_;
typedef struct tp_dict_ {
    enum TP_OBJ_TYPE type;
    struct _tp_dict *val;
} tp_dict_;
typedef struct tp_fnc_ {
    enum TP_OBJ_TYPE type;
    struct _tp_fnc *info;
    int ftype;
    void *val;
} tp_fnc_;
typedef struct tp_data_ {
    enum TP_OBJ_TYPE type;
    struct _tp_data *info;
    void *val;
    int magic;
} tp_data_;

typedef union tp_obj {
    enum TP_OBJ_TYPE type;
    tp_number_ number;
    struct { enum TP_OBJ_TYPE type; int *data; } gci;
    tp_string_ string;
    tp_dict_ dict;
    tp_list_ list;
    tp_fnc_ fnc;
    tp_data_ data;
} tp_obj;

typedef struct _tp_string {
    int gci;
    char s[1];
} _tp_string;
typedef struct _tp_list {
    int gci;
    tp_obj *items;
    int len;
    int alloc;
} _tp_list;
typedef struct tp_item {
    int used;
    int hash;
    tp_obj key;
    tp_obj val;
} tp_item;
typedef struct _tp_dict {
    int gci;
    tp_item *items;
    int len;
    int alloc;
    int cur;
    int mask;
    int used;
} _tp_dict;
typedef struct _tp_fnc {
    int gci;
    tp_obj self;
    tp_obj globals;
} _tp_fnc;


typedef union tp_code {
    unsigned char i;
    struct { unsigned char i,a,b,c; } regs;
    struct { char val[4]; } string;
    struct { float val; } number;
} tp_code;

typedef struct tp_frame_ {
    tp_code *codes;
    tp_code *cur;
    tp_code *jmp;
    tp_obj *regs;
    tp_obj *ret_dest;
    tp_obj fname;
    tp_obj name;
    tp_obj line;
    tp_obj globals;
    int lineno;
    int cregs;
} tp_frame_;

typedef int (*tp_exception_cb)(int result);

#define TP_GCMAX 4096
#define TP_FRAMES 256
/* #define TP_REGS_PER_FRAME 256*/
#define TP_REGS 16384
typedef struct tp_vm {
    tp_obj builtins;
    tp_obj modules;
    tp_frame_ frames[TP_FRAMES];
    tp_obj _params;
    tp_obj params;
    tp_obj _regs;
    tp_obj *regs;
    tp_obj root;
    jmp_buf buf;
    int jmp;
    tp_obj ex;
    char chars[256][2];
    int cur;
    /* gc*/
    _tp_list *white;
    _tp_list *grey;
    _tp_list *black;
    _tp_dict *strings;
    int steps;
    tp_exception_cb exception_function;  /* client callback function */
} tp_vm;

#define TP tp_vm *tp
typedef struct _tp_data {
    int gci;
    void (*free)(TP,tp_obj);
} _tp_data;

/* NOTE: these are the few out of namespace items for convenience*/
#define tp_True tp_number(1)
#define tp_False tp_number(0)
#define TP_CSTR(v) ((tp_str(tp,(v))).string.val)

extern tp_obj tp_None;

#ifdef __cplusplus
extern "C"
{
#endif

/* list.c */
tp_obj _tp_list_get(TP,_tp_list *self,int k,const char *error);
tp_obj _tp_list_copy(TP, tp_obj rr);
tp_obj tp_list(TP);
void _tp_list_append(TP,_tp_list *self, tp_obj v);
void _tp_list_appendx(TP,_tp_list *self, tp_obj v);
_tp_list *_tp_list_new(void);
void _tp_list_free(_tp_list *self);
tp_obj _tp_list_pop(TP,_tp_list *self, int n, const char *error);
int _tp_list_find(TP,_tp_list *self, tp_obj v);
void _tp_list_insert(TP,_tp_list *self, int n, tp_obj v);
tp_obj tp_list_n(TP,int n,tp_obj *argv);
void _tp_list_set(TP,_tp_list *self,int k, tp_obj v, const char *error);
tp_obj tp_append(TP);
tp_obj tp_pop(TP);
tp_obj tp_index(TP);
tp_obj tp_sort(TP);
tp_obj tp_extend(TP);

/* misc.c */
tp_obj tp_fnc_new(TP,int t, void *v, tp_obj s, tp_obj g);
tp_obj tp_method(TP,tp_obj self,tp_obj v(TP));
tp_obj tp_params_v(TP,int n,...);
tp_obj _tp_tcall(TP,tp_obj fnc);
tp_obj tp_params(TP);
tp_obj tp_params_n(TP,int n, tp_obj argv[]);
tp_obj tp_def(TP,void *v, tp_obj g);
tp_obj tp_data(TP,int magic,void *v);
tp_obj tp_fnc(TP,tp_obj v(TP));

/* dict.c */
tp_obj _tp_dict_copy(TP,tp_obj rr);
_tp_dict *_tp_dict_new(void);
int _tp_dict_next(TP,_tp_dict *self);
void _tp_dict_free(_tp_dict *self);
void _tp_dict_del(TP,_tp_dict *self,tp_obj k, const char *error);
int _tp_dict_find(TP,_tp_dict *self,tp_obj k);
void _tp_dict_setx(TP,_tp_dict *self,tp_obj k, tp_obj v);
tp_obj _tp_dict_get(TP,_tp_dict *self,tp_obj k, const char *error);
void _tp_dict_set(TP,_tp_dict *self,tp_obj k, tp_obj v);
tp_obj tp_dict(TP);
tp_obj tp_dict_n(TP,int n, tp_obj* argv);
tp_obj tp_merge(TP);

/* string.c */
tp_obj tp_string_t(TP, int n);
tp_obj tp_join(TP);
tp_obj tp_split(TP);
tp_obj tp_strip(TP);
tp_obj tp_str_index(TP);
tp_obj tp_replace(TP);
tp_obj tp_str2(TP);
tp_obj tp_chr(TP);
tp_obj tp_ord(TP);

/* builtins.c */
tp_obj tp_copy(TP);
tp_obj tp_print(TP);
tp_obj tp_load(TP);
tp_obj tp_range(TP);
tp_obj tp_min(TP);
tp_obj tp_max(TP);
tp_obj tp_bind(TP);
tp_obj tp_len_(TP);
tp_obj tp_float(TP);
tp_obj tp_system(TP);
tp_obj tp_istype(TP);
tp_obj tp_save(TP);
tp_obj tp_fpack(TP);
tp_obj tp_abs(TP);
tp_obj tp_int(TP);
tp_obj tp_round(TP);
tp_obj tp_exists(TP);
tp_obj tp_mtime(TP);

/* vm.c */
tp_obj tp_import(TP, char const *fname, char const *name, void *codes);
tp_obj tp_call(TP, const char *mod, const char *fnc, tp_obj params);
tp_vm *tp_init(int argc, char *argv[], tp_exception_cb function);
void tp_deinit(TP);

/* gc.c */
void tp_gc_init(TP);
void tp_full(TP);
void tp_delete(TP,tp_obj v);
void tp_gc_deinit(TP);

/* ops.c */
tp_obj tp_add(TP,tp_obj a, tp_obj b);
tp_obj tp_mul(TP,tp_obj a, tp_obj b);
tp_obj tp_sub(TP,tp_obj a, tp_obj b);
tp_obj tp_div(TP,tp_obj a, tp_obj b);
tp_obj tp_pow(TP,tp_obj a, tp_obj b);
tp_obj tp_and(TP,tp_obj a, tp_obj b);
tp_obj tp_or(TP,tp_obj a, tp_obj b);
tp_obj tp_mod(TP,tp_obj a, tp_obj b);
tp_obj tp_lsh(TP,tp_obj a, tp_obj b);
tp_obj tp_rsh(TP,tp_obj a, tp_obj b);
tp_obj tp_len(TP,tp_obj self);
int tp_bool(TP,tp_obj v);
tp_obj tp_str(TP,tp_obj self);
int tp_iget(TP,tp_obj *r, tp_obj self, tp_obj k);
void tp_del(TP,tp_obj self, tp_obj k);
tp_obj tp_has(TP,tp_obj self, tp_obj k);
tp_obj tp_iter(TP,tp_obj self, tp_obj k);
tp_obj tp_assert(TP);
void tp_set(TP,tp_obj self, tp_obj k, tp_obj v);

/* tp.c */
void tp_compiler(TP);

tp_obj tp_get(TP,tp_obj,tp_obj);
int tp_cmp(TP,tp_obj,tp_obj);
void _tp_raise(TP,tp_obj);
tp_obj tp_printf(TP,char const *fmt,...);
tp_obj tp_track(TP,tp_obj);
void tp_grey(TP,tp_obj);

/* __func__ __VA_ARGS__ __FILE__ __LINE__ */
#define tp_raise(r,fmt,...) { \
    _tp_raise(tp,tp_printf(tp,fmt,__VA_ARGS__)); \
    return r; \
}
#define TP_OBJ() (tp_get(tp,tp->params,tp_None))
tp_inline static tp_obj tp_type(TP,int t,tp_obj v) {
    if (v.type != t) { tp_raise(tp_None,"_tp_type(%d,%s)",t,TP_CSTR(v)); }
    return v;
}
#define TP_TYPE(t) tp_type(tp,t,TP_OBJ())
#define TP_NUM() (TP_TYPE(TP_NUMBER).number.val)
#define TP_STR() (TP_CSTR(TP_TYPE(TP_STRING)))
#define TP_DEFAULT(d) (tp->params.list.val->len?tp_get(tp,tp->params,tp_None):(d))
#define TP_LOOP(e) \
    int __l = tp->params.list.val->len; \
    int __i; for (__i=0; __i<__l; __i++) { \
    (e) = _tp_list_get(tp,tp->params.list.val,__i,"TP_LOOP");
#define TP_END \
    }

tp_inline static int _tp_min(int a, int b) { return (a<b?a:b); }
tp_inline static int _tp_max(int a, int b) { return (a>b?a:b); }
tp_inline static int _tp_sign(tp_num v) { return (v<0?-1:(v>0?1:0)); }

tp_inline static tp_obj tp_number(tp_num v) {
    tp_obj val = {TP_NUMBER};
    val.number.val = v;
    return val;
}

tp_inline static tp_obj tp_string(char const *v) {
    tp_obj val;
    tp_string_ s = {TP_STRING, 0, v, 0};
    s.len = strlen(v);
    val.string = s;
    return val;
}

tp_inline static tp_obj tp_string_n(char const *v,int n) {
    tp_obj val;
    tp_string_ s = {TP_STRING, 0,v,n};
    val.string = s;
    return val;
}

/* modules */

/* rtstepper/init.c */
void rtstepperpy_init(struct tp_vm *tp);

#ifdef __cplusplus
}
#endif

/* bc.c */
extern unsigned char tp_tokenize[];
extern unsigned char tp_parse[];
extern unsigned char tp_encode[];
extern unsigned char tp_py2bc[];

#endif  /* _TINYPY_H */
