import sys

def main():
    out = []
    mod = sys.argv[1]
    out.append("""unsigned char tp_%s[] = {"""%mod)
    fname = "tinypy/"+mod+".tpc"
    data = open(fname,'rb').read()
    cols = 16
    for n in xrange(0,len(data),cols):
        out.append(",".join([str(ord(v)) for v in data[n:n+cols]])+',')
    out.append("""};""")
    out.append("")
    f = open('bc.c','ab')
    f.write('\n'.join(out))
    f.close()

main()
