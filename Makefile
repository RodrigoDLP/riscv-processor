# =========================================================
# Herramientas y Nombres de Archivos por Defecto
# =========================================================
IVERILOG = iverilog
VVP = vvp
GTKWAVE = gtkwave

VCD_FILE = test.vcd

# =========================================================
# Definición de los Archivos Fuente para cada Prueba
# =========================================================

# Archivos de diseño para la prueba de la ALU
PIPE_DESIGN_FILES = processor.v

# Testbench para la ALU
PIPE_TB_FILE = testbench.v

# Archivos de diseño para la prueba del TOP (incluye todo el diseño)
#TOP_DESIGN_FILES = top.v $(ALU_DESIGN_FILES)

# Testbench para el TOP
#TOP_TB_FILE = top_tb.v

# =========================================================
# Objetivos Principales (Lo que escribes en la terminal)
# =========================================================
.PHONY: all test_pipe view clean #test_top view clean

# El objetivo por defecto será probar el módulo TOP
all: test_pipe

# --- Objetivo para probar solo la ALU ---
test_pipe:
	$(IVERILOG) -o pipe_test $(PIPE_DESIGN_FILES) $(PIPE_TB_FILE)
	$(VVP) pipe_test

# --- Objetivo para probar el sistema completo (TOP) ---
#test_top:
#	$(IVERILOG) -o top_test $(TOP_DESIGN_FILES) $(TOP_TB_FILE)
#	$(VVP) top_test

# =========================================================
# Objetivos Auxiliares
# =========================================================

# Regla para abrir GTKWave con el archivo de ondas
# Se asume que tu testbench tiene la línea `$dumpfile("waves.vcd");`
view:
	$(GTKWAVE) $(VCD_FILE) &

# Regla para limpiar todos los archivos generados por ambas pruebas
clean:
	rm -f pipe_test $(VCD_FILE)
