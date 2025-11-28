module EXEC #(
    parameter ELEMENTS_PER_REGISTER = 4,   // Número de elementos por vector / Número de ALUs (lanes)
    parameter ELEM_WIDTH = 32              // Ancho de cada elemento
) (
    // Entradas
    input clk,
    input reset,
    input [ELEMENTS_PER_REGISTER*ELEM_WIDTH - 1:0] vec_op_A, // Vector Operando A (desde RVF rd1)
    input [ELEMENTS_PER_REGISTER*ELEM_WIDTH - 1:0] vec_op_B, // Vector Operando B (desde RVF rd2)
    input [3:0] op_code,                                    // Código de operación (ej. 4'b0001 para VADD)
    input vector_enable,                                    // Señal de habilitación vectorial

    // Salidas
    output reg [ELEMENTS_PER_REGISTER*ELEM_WIDTH - 1:0] vec_result_out // Vector Resultado (hacia el RVF wd3)
);

    // Cálculos de ancho
    localparam VR_WIDTH = ELEMENTS_PER_REGISTER * ELEM_WIDTH;
    
    //----------------------------------------------------------------------
    // 1. Cables intermedios para la conexión de los Lanes (ALUs)
    //----------------------------------------------------------------------
    // Dividir los vectores de entrada en elementos individuales
    wire [ELEM_WIDTH-1:0] element_A [0:ELEMENTS_PER_REGISTER-1];
    wire [ELEM_WIDTH-1:0] element_B [0:ELEMENTS_PER_REGISTER-1];
    
    // Resultados de cada Lane (ALU individual)
    wire [ELEM_WIDTH-1:0] lane_result [0:ELEMENTS_PER_REGISTER-1];

    // Desempaquetar los vectores grandes en elementos individuales
    // Esto es crucial para conectar cada elemento al bus de datos de su ALU.
    generate
        genvar i;
        for (i = 0; i < ELEMENTS_PER_REGISTER; i = i + 1) begin
            // Extraer el i-ésimo elemento de A y B
            assign element_A[i] = vec_op_A[ELEM_WIDTH*(i+1)-1 : ELEM_WIDTH*i];
            assign element_B[i] = vec_op_B[ELEM_WIDTH*(i+1)-1 : ELEM_WIDTH*i];
            
            //------------------------------------------------------------------
            // Instanciación del Lane (ALU Escalar)
            //------------------------------------------------------------------
            // Cada Lane opera en un par de elementos A[i] y B[i] de forma independiente.
            mALUma Lane_i (
                .clk(clk)
                .op_A(element_A[i]),
                .op_B(element_B[i]),
                .op_code(op_code), // Todas las lanes reciben el mismo op_code
                .result(lane_result[i])
            );
        end
    endgenerate

    //----------------------------------------------------------------------
    // 2. Registro de Salida
    //----------------------------------------------------------------------
    // Empaquetar los resultados de los Lanes en un único vector de salida.
    // Esto simula la fase de Write Back Vectorial (aunque aquí es sólo una caja de registro).
    always @(posedge clk) begin
        if (reset) begin
            vec_result_out <= ZERO_VECTOR;
        end else if (vector_enable) begin
            // Empaquetar los resultados de todos los lanes en el vector de salida
            // Usamos una concatenación directa para simplificar el hardware de empaquetado.
            vec_result_out <= {lane_result};
        end
    end
    
    //----------------------------------------------------------------------
    // 3. Definición del Vector de Ceros
    //----------------------------------------------------------------------
    localparam ZERO_VECTOR = {VR_WIDTH{1'b0}};

endmodule
