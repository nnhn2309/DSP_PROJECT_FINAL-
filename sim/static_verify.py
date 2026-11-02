#!/usr/bin/env python3
"""Project-level static checks. Does NOT replace compiling HDL using Quartus/iverilog."""
import csv,re,pathlib,sys,json
R=pathlib.Path(__file__).resolve().parents[1]
results=[]
def test(name,pred,details=''):
    results.append(dict(test=name,pass_=bool(pred),details=str(details)))
    print(('PASS ' if pred else 'FAIL ')+name+': '+str(details))
    if not pred: raise AssertionError(name)

files=list((R/'rtl').glob('*.v'))
sources={f.stem:f.read_text() for f in files}
test('RTL modules present',len(sources)==9,sorted(sources))
# Simple lexical delimiter checks on Verilog stripped of comments and strings.
for name,src in sources.items():
    clean=re.sub(r'/\*.*?\*/|//[^\n]*','',src,flags=re.S)
    clean=re.sub(r'"(?:\\.|[^"\\])*"','""',clean)
    for a,b in [('module','endmodule'),('begin','end'),('case','endcase'),('function','endfunction')]:
        counta=len(re.findall(r'\b'+a+r'\b',clean));countb=len(re.findall(r'\b'+b+r'\b',clean))
        test(f'{name} {a}/{b} count',counta==countb,f'{counta}/{countb}')
    for x,y in [('(',')'),('[',']')]:
        # Ignore preprocessor strings and bit syntax (no parentheses in comments)
        test(f'{name} braces {x}{y}',clean.count(x)==clean.count(y),f'{clean.count(x)}/{clean.count(y)}')

# Check QSF source filenames and location assignments exactly match reference CSV.
qsf=(R/'quartus/de2_anc.qsf').read_text()
qfiles=re.findall(r'VERILOG_FILE\s+\.\./rtl/(\S+)',qsf)
test('Quartus references all sources',set(qfiles)==set(f.name for f in files),sorted(qfiles))
rows={row[0]:row[1] for row in csv.reader(open('/mnt/data/DE2_pin_assignments.csv')) if len(row)==2 and row[1].startswith('PIN_')}
pairs=re.findall(r'set_location_assignment\s+(PIN_\S+)\s+-to\s+(\S+)',qsf)
test('QSF 45 pins unique',len(pairs)==45 and len(set(x for x,y in pairs))==45 and len(set(y for x,y in pairs))==45,len(pairs))
test('QSF pins match DE2 CSV',all(rows.get(sig)==pin for pin,sig in pairs),'45 assignments')

core=sources['feedback_fxlms_bram']
for key in ['s_ram','w_ram','v_hist','y_hist','x_hist','xf_hist']:
    test(f'{key} M4K style',bool(re.search(r'ramstyle\s*=\s*"M4K"[^;]*\b'+key+r'\[',core)),'synchronous BRAM-style declaration')
    test(f'{key} not array async-reset',not bool(re.search(key+r'\[\s*i\s*\]\s*<=',core)), 'no for-loop clear in async-reset')
test('No old 16-bit xf_new',not re.search(r'reg\s+signed\s+\[15:0\]\s+xf_new',core),'32-bit xf_history')
test('Offline S frozen in RUN','if(state==S_COMMIT)' in core and 'cal_mode==CAL_TRAIN' in core,'S RAM only written by S_COMMIT and sequential CLEAR')
rom=re.findall(r"7'd\d+: coeff\s*=",sources['decimator8_fir127'])
test('FIR ROM length',len(rom)==127,len(rom))
DSP_MAX=3*128*7+64*7+64*6+150
budget=50_000_000//6000
test('Nominal cycles fit 6kHz',DSP_MAX<budget,f'est. {DSP_MAX} < budget {budget}, excludes fitter delays')
(R/'results/static_test_results.json').write_text(json.dumps(results,indent=2,ensure_ascii=False))
print('PASS total',len(results))
