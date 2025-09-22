#include <stdio.h>
#include <stdbool.h>

int lookup_table[26][5] =   {
                            {65,1,2,0,0},     // A: · –
                            {66,2,1,1,1},     // B: – · · ·
                            {67,2,1,2,1},     // C: – · – ·
                            {68,2,1,1,0},     // D: – · ·
                            {69,1,0,0,0},     // E: ·
                            {70,1,1,2,1},     // F: · · – ·
                            {71,2,2,1,0},     // G: – – ·
                            {72,1,1,1,1},     // H: · · · ·
                            {73,1,1,0,0},     // I: · ·
                            {74,1,2,2,2},     // J: · – – –
                            {75,2,1,2,0},     // K: – · –
                            {76,1,2,1,1},     // L: · – · ·
                            {77,2,2,0,0},     // M: – –
                            {78,2,1,0,0},     // N: – ·
                            {79,2,2,2,0},     // O: – – –
                            {80,1,2,2,1},     // P: · – – ·
                            {81,2,2,1,2},     // Q: – – · –
                            {82,1,2,1,0},     // R: · – ·
                            {83,1,1,1,0},     // S: · · ·
                            {84,2,0,0,0},     // T: –
                            {85,1,1,2,0},     // U: · · –
                            {86,1,1,1,2},     // V: · · · –
                            {87,1,2,2,0},     // W: · – –
                            {88,2,1,1,2},     // X: – · · –
                            {89,2,1,2,2},     // Y: – · – –
                            {90,2,2,1,1}      // Z: – – · ·
                            };


int main (void)
{
    // Display morse code
    char input_char = 'S';
    int row_match = 0;
    bool match_found = false;

    for (int i = 0; i < 26; i++)                    // For each row
    {
        // Look for match in the table
        if (lookup_table[i][0] == input_char)
        {
            row_match = i;
            match_found = true;
            break;
        }
    }
    
    if (match_found)
    {
        //Display the dot dash pattern
        for (int i = 1; i < 5; i++)
        {
            int value = lookup_table[row_match][i];
            if (value == 1)
            {
                printf("Dot\n");
            }
            if (value == 2)
            {
                printf("Dash\n");
            }
        }
    }
}