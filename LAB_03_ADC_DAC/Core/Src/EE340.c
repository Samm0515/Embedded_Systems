/*
 * EE340.c
 *
 *	LAB03_EE340
 *
 *  Created on: Oct 6, 2025
 *  Author: Samuel Meysembourg
 *
 */


void processADC(void)
{
	//TODO
}













/*
// Code for making circular buffer with a linked list

typedef struct node
{
    int32_t value;
    struct node* next;
}node_t;

// Function Prototypes
node_t* CreateBuffer(void);
void UpdateBuffer(node_t* head, int32_t new_value);
void PrintBuffer(node_t* head);
float BufferAverage(node_t* head);


int main (void)
{
    // Create HEAD
    node_t* BUFFER;
    BUFFER = CreateBuffer();

    // Add elements to head
    for (int i = 1; i <= 100; i++)
    {
        UpdateBuffer(BUFFER, i);
    }
    PrintBuffer(BUFFER);
    printf("Buffer Average : %.2f\n", BufferAverage(BUFFER));

    for

}



node_t* CreateBuffer(void)
{
    node_t* temp_head = (node_t*)malloc(sizeof(node_t));

    if (temp_head != NULL)
    {
        temp_head -> next = NULL;
        temp_head -> value = 0;
        return temp_head;
    }
    else
    {
        free(temp_head);
        return NULL;
    }
}

void UpdateBuffer(node_t* head, int32_t new_value)
{
    // Check if this is the head
    node_t* current = head;
    if(!(head->next == NULL))
    {
        // Find the end of the list
        while (current->next != NULL)
        {
            current = current->next;
        }
    }

    // Add new node to buffer
    node_t* new_node = (node_t*)malloc(sizeof(node_t));

    if (new_node != NULL)
    {
        // Add new data to new
        new_node->value = new_value;
        // Assign to end of buffer
        current->next = new_node;
        // Assign the end of the buffer
        new_node->next = NULL;
    }
}

void PrintBuffer(node_t* head)
{
    node_t* current = head;
    while (current != NULL)
    {
        printf("%d\n",current->value);
        current = current->next;
    }
}

float BufferAverage(node_t* head)
{
    node_t* current = head;
    int64_t total = 0;
    int32_t count = 0;
    while (current != NULL)
    {
        total += current->value;
        count++;
        current = current->next;
    }
    return (float)total / count;
}
*/
