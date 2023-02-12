// servermain.c

#include "common.h"

static void usage(void)
{
	printf("Usage: jtagserver -c --client\n");
}

int main (int argc, char **argv)
{
	g_isserver = 1;
	g_standalone = 0;

	if (argc == 2 && argv && argv[1])
	{
		if (!strcmp(argv[1], "-c") || !strcmp(argv[1], "--client"))
		{
			g_isserver = 0;
			printf("running as test client\n");
		}
		else
		{
			usage();
			return 1;
		}	
	}
	else if (argc > 2)
	{
		usage();
		return 1;
	}

	if (initmessage())
		return 1;

	if (g_isserver)
		server();
	else
		client();

	return 0;
}
