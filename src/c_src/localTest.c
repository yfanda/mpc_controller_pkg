#include <stdio.h>
#include <stdlib.h>
#pragma warning(disable:4996)
int myTest()
{
    int i, n;
    int* a;

    printf("Ҫ�����Ԫ�ظ�����");
    scanf("%d", &n);

    a = (int*)calloc(n, sizeof(int));
    printf("���� %d �����֣�\n", n);
    for (i = 0; i < n; i++)
    {
        scanf("%d", &a[i]);
    }

    printf("���������Ϊ��");
    for (i = 0; i < n; i++) {
        printf("%d ", a[i]);
    }
    free(a);  // �ͷ��ڴ�
    return(0);
}