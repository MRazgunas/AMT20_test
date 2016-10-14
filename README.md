ECU module firmware. Now it is prototype on other board. Board will be designed by the end of year

<h1>Adding new parameters</h1>
<ol>
<li>Add new parameter to EEPROM layout enum (located in parameters_d.h) using this format: k_param_xx (DO NOT CHANGE EXISTING ORDER!!!)</li>
<li>Define new variable using xx name in parameters_d.h as extern and in parameters_d.c (supported types int8_t, int16_t, int32_t and float)</li>
<li>Add new parameter to var_list array located in parameters_d.c using this format: GSCALAR(param_type, xx, "PARAM_NAME", def_value)</li>
</ol>
