import re
import os

tex_files = [
    '/Users/leonardoleggeri/Desktop/Books/Tesi/capitoli/appendice_a.tex',
    '/Users/leonardoleggeri/Desktop/Books/Tesi/capitoli/appendice_b.tex'
]

code_dir = '/Users/leonardoleggeri/Desktop/PROGETTI/Automazione/AutomationF16/Code/Studio_MIMO/F16MIMO'

file_mapping = {
    'lst:discretizza_modello': 'funzioni_mpc/discretizza_modello.m',
    'lst:cis': 'funzioni_mpc/cis.m',
    'lst:plot_cis': 'funzioni_mpc/plot_cis.m'
}

for tex_file in tex_files:
    with open(tex_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    for label, rel_path in file_mapping.items():
        if label not in content:
            continue
            
        src_path = os.path.join(code_dir, rel_path)
        if not os.path.exists(src_path):
            print(f"Warning: {src_path} not found!")
            continue
            
        with open(src_path, 'r', encoding='utf-8') as f:
            src_code = f.read().strip()
            
        # Regex to find the block
        pattern = r'(\\begin\{lstlisting\}\[.*?label=\{?' + re.escape(label) + r'\}?.*?\].*?\n)(.*?)(\\end\{lstlisting\})'
        
        def replacer(match):
            return match.group(1) + src_code + '\n' + match.group(3)
            
        new_content = re.sub(pattern, replacer, content, flags=re.DOTALL)
        content = new_content
        print(f"Updated {label} in {os.path.basename(tex_file)}")
        
    with open(tex_file, 'w', encoding='utf-8') as f:
        f.write(content)

