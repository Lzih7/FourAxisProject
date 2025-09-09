;systick 延迟函数
	THUMB
	PRESERVE8
	AREA |.text|,CODE,READONLY,ALIGN=2
		
	;导出函数
	EXPORT systick_init
	EXPORT delay_ms
		
;systick寄存器地址
SYST_CSR EQU 0xE000E010 ;配置时钟源 中断 使能
SYST_RVR EQU 0xE000E014 ;决定定时器的周期
SYST_CVR EQU 0xE000E018 ;读取当前计数
	
; 系统时钟频率
SystemCoreClock EQU 72000000 ;72MHz
	
systick_init PROC ;标识子程序的开始
	LDR R0,=SYST_CSR
	LDR R1,[R0]
	BIC R1,0x00000001 ;清除使能位
	STR R1,[R0]
	
	;设置时钟源为处理器时钟
	LDR R1,[R0]
	ORR R1,#0x00000004 ;设置CLKSOURCE位
	STR R1,[R0]
	
	;禁用中断
	LDR R1,[R0]
	BIC R1,#0x00000002 ;清除TICKINT位
	STR R1,[R0]
	
	BX LR
	ENDP
		
;毫秒延迟
Delay_ms PROC
	PUSH {R1-R4,LR}
	
	LDR R1,=SystemCoreClock
	MOV R2,#1000
	UDIV R1,R1,R2 ;R1=SystemCoreClock/1000
	SUBS R1,R1,#1 ;计数次数=初始值N+1
	
	;设置systick
	LDR R2,=SYST_RVR
	STR R1,[R2] ;将计算好的周期写入systick中以设定重载值 
	
	LDR R2,=SYST_CVR
	MOV R1,#0 ; 写入以清空CVR
	STR R1,[R2]
	
	LDR R2,=SYST_CSR
	LDR R1,[R2] ;使用内核时钟源
	ORR R1,R1,#0x00000005 ; 设置CLKSOURCE和ENABLE位 
	STR R1,[R2] ; 不设置TICKINT

delay_loop 
	LDR R1,=SYST_CSR
	LDR R1,[R1] ;BEQ指定检查Z标志位 Z由最后设置s后缀的指令（ANDS）给出
	ANDS R1,R1,#0x00010000 ;检查COUNTFLAG位
	BEQ delay_loop ; 当R1为0时退出