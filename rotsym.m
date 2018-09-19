function R=rotsym(a,b)
    c=cos(b);
    s=sin(b);
    if (a=='x')

        R=[1  0  0;
            0  c -s;
            0  s  c];
    end;
    if (a=='y')

        R=[ c  0  s;
            0  1  0;
            -s  0  c];
    end;
    if (a=='z')

        R=[c -s  0;
            s  c  0;
            0  0  1];
    end;
end

   