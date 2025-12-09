void Task_SampleINA(void)
{
    float v, i, p;

    /* --- INA219 #1 --- */
    INA_Read(INA219_ADDR1, &v, &i, &p);
    ch1.v_V = v;
    ch1.i_A = i;
    ch1.p_W = p;

    /* --- INA219 #2 --- */
    INA_Read(INA219_ADDR2, &v, &i, &p);
    ch2.v_V = v;
    ch2.i_A = i;
    ch2.p_W = p;

    /* --- LIVE SERIAL OUTPUT --- */
    char msg[128];
    snprintf(msg, sizeof(msg),
             "CH1: %.3f mA   CH2: %.3f mA\r\n",
             ch1.i_A * 1000.0f,
             ch2.i_A * 1000.0f);

    debug_print(msg);
}
