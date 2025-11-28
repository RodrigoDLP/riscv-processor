module RVF #(
    parameter ELEMENTS_PER_REGISTER = 4,   // Número de elementos por registro vectorial (VL)
    parameter ELEM_WIDTH = 32              // Ancho de cada elemento en bits
) (
    input clk,
    input reset,

    // Puerto de escritura (para el resultado de la Unidad EXEC Vectorial)
    input we3,                           // Habilitación de escritura
    input [4:0] a3,                      // Dirección de escritura (VR0 a VR31)
    input [ELEM_WIDTH*ELEMENTS_PER_REGISTER - 1:0] wd3, // Dato de escritura (Vector completo)

    // Puertos de lectura (para los operandos de la Unidad EXEC Vectorial)
    input [4:0] a1,                      // Dirección de lectura 1
    input [4:0] a2,                      // Dirección de lectura 2
    output [ELEM_WIDTH*ELEMENTS_PER_REGISTER - 1:0] rd1, // Dato de lectura 1 (Vector completo)
    output [ELEM_WIDTH*ELEMENTS_PER_REGISTER - 1:0] rd2  // Dato de lectura 2 (Vector completo)
);

    // Cálculos de ancho y profundidad
    localparam VR_DEPTH = 32;
    localparam VR_WIDTH = ELEMENTS_PER_REGISTER * ELEM_WIDTH;
    localparam ZERO_VECTOR = {VR_WIDTH{1'b0}}; // Un vector de ceros de ancho adecuado

    // El almacenamiento interno: array de registros vectoriales
    // rf[31:0] almacena 32 registros, cada uno de VR_WIDTH bits de ancho.
    reg [VR_WIDTH-1:0] rf [0:VR_DEPTH-1];

    // Puerto de Escritura (Escritura de un vector completo)
    // Se escribe en el flanco de bajada (negedge clk)
    integer i;
    always @(negedge clk) begin
        if (reset) begin
            // Inicializar todos los registros a cero al resetear
            for (i = 0; i < VR_DEPTH; i = i + 1) begin
                rf[i] <= ZERO_VECTOR;
            end
        end 
        // Lógica de escritura: solo se escribe si we3 está activo Y el registro de destino NO es VR0 (a3 != 0)
        else if (we3 && (a3 != 5'b0)) begin 
            rf[a3] <= wd3;
        end
    end

    // Puertos de Lectura (Lectura de un vector completo)
    // Lectura combinacional. Si la dirección es 0, devuelve el vector de ceros.
    assign rd1 = (a1 != 5'b0) ? rf[a1] : ZERO_VECTOR;
    assign rd2 = (a2 != 5'b0) ? rf[a2] : ZERO_VECTOR;

endmodule
