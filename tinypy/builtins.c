#include "tinypy.h"

tp_obj tp_print(TP) {
    int n = 0;
    tp_obj e;
    TP_LOOP(e)
        if (n) { printf(" "); }
        printf("%s",TP_CSTR(e));
        n += 1;
    TP_END;
    printf("\n");
    return tp_None;
}

tp_obj tp_bind(TP) {
    tp_obj r = TP_OBJ();
    tp_obj self = TP_OBJ();
    return tp_fnc_new(tp,r.fnc.ftype|2,r.fnc.val,self,r.fnc.info->globals);
}

tp_obj tp_min(TP) {
    tp_obj r = TP_OBJ();
    tp_obj e;
    TP_LOOP(e)
        if (tp_cmp(tp,r,e) > 0) { r = e; }
    TP_END;
    return r;
}

tp_obj tp_max(TP) {
    tp_obj r = TP_OBJ();
    tp_obj e;
    TP_LOOP(e)
        if (tp_cmp(tp,r,e) < 0) { r = e; }
    TP_END;
    return r;
}

tp_obj tp_copy(TP) {
    tp_obj r = TP_OBJ();
    int type = r.type;
    if (type == TP_LIST) {
        return _tp_list_copy(tp,r);
    } else if (type == TP_DICT) {
        return _tp_dict_copy(tp,r);
    }
    tp_raise(tp_None,"tp_copy(%s)",TP_CSTR(r));
}


tp_obj tp_len_(TP) {
    tp_obj e = TP_OBJ();
    return tp_len(tp,e);
}


tp_obj tp_assert(TP) {
    int a = TP_NUM();
    if (a) { return tp_None; }
    tp_raise(tp_None,"%s","assert failed");
}

tp_obj tp_range(TP) {
    int a,b,c,i;
    tp_obj r = tp_list(tp);
    switch (tp->params.list.val->len) {
        case 1: a = 0; b = TP_NUM(); c = 1; break;
        case 2:
        case 3: a = TP_NUM(); b = TP_NUM(); c = TP_DEFAULT(tp_number(1)).number.val; break;
        default: return r;
    }
    if (c != 0) {
        for (i=a; (c>0) ? i<b : i>b; i+=c) {
            _tp_list_append(tp,r.list.val,tp_number(i));
        }
    }
    return r;
}


tp_obj tp_system(TP) {
    char const *s = TP_STR();
    int r = system(s);
    return tp_number(r);
}

tp_obj tp_istype(TP) {
    tp_obj v = TP_OBJ();
    char const *t = TP_STR();
    if (strcmp("string",t) == 0) { return tp_number(v.type == TP_STRING); }
    if (strcmp("list",t) == 0) { return tp_number(v.type == TP_LIST); }
    if (strcmp("dict",t) == 0) { return tp_number(v.type == TP_DICT); }
    if (strcmp("number",t) == 0) { return tp_number(v.type == TP_NUMBER); }
    tp_raise(tp_None,"is_type(%s,%s)",TP_CSTR(v),t);
}


tp_obj tp_float(TP) {
    tp_obj v = TP_OBJ();
    int ord = TP_DEFAULT(tp_number(0)).number.val;
    int type = v.type;
    if (type == TP_NUMBER) { return v; }
    if (type == TP_STRING) {
        if (strchr(TP_CSTR(v),'.')) { return tp_number(atof(TP_CSTR(v))); }
        return(tp_number(strtol(TP_CSTR(v),0,ord)));
    }
    tp_raise(tp_None,"tp_float(%s)",TP_CSTR(v));
}


tp_obj tp_save(TP) {
    char const *fname = TP_STR();
    tp_obj v = TP_OBJ();
    FILE *f;
    f = fopen(fname,"wb");
    if (!f) { tp_raise(tp_None,"tp_save(%s,...)",fname); }
    fwrite(v.string.val,v.string.len,1,f);
    fclose(f);
    return tp_None;
}

tp_obj tp_load(TP) {
    FILE *f;
    long l;
    tp_obj r;
    char *s;
    char const *fname = TP_STR();
    struct stat stbuf;
    stat(fname, &stbuf);
    l = stbuf.st_size;
    f = fopen(fname,"rb");
    if (!f) {
        tp_raise(tp_None,"tp_load(%s)",fname);
    }
    r = tp_string_t(tp,l);
    s = r.string.info->s;
    fread(s,1,l,f);
    fclose(f);
    return tp_track(tp,r);
}


tp_obj tp_fpack(TP) {
    tp_num v = TP_NUM();
    tp_obj r = tp_string_t(tp,sizeof(tp_num));
    *(tp_num*)r.string.val = v;
    return tp_track(tp,r);
}

tp_obj tp_abs(TP) {
    return tp_number(fabs(tp_float(tp).number.val));
}
tp_obj tp_int(TP) {
    return tp_number((long)tp_float(tp).number.val);
}
tp_num _roundf(tp_num v) {
    tp_num av = fabs(v); tp_num iv = (long)av;
    av = (av-iv < 0.5?iv:iv+1);
    return (v<0?-av:av);
}
tp_obj tp_round(TP) {
    return tp_number(_roundf(tp_float(tp).number.val));
}

tp_obj tp_exists(TP) {
    char const *s = TP_STR();
    struct stat stbuf;
    return tp_number(!stat(s,&stbuf));
}
tp_obj tp_mtime(TP) {
    char const *s = TP_STR();
    struct stat stbuf;
    if (!stat(s,&stbuf)) { return tp_number(stbuf.st_mtime); }
    tp_raise(tp_None,"tp_mtime(%s)",s);
}
