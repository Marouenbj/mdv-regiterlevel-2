#pragma once
#ifndef COMMAND_H
#define COMMAND_H

#include "mdv.h"

__STATIC_FORCEINLINE void
command_check (fsm_t *fsm)
{
  char *p_param = NULL;
  char *saveptr = NULL;

  // check our parameters
  p_param = strtok_r ((void *)fsm->buf.mainbuf_ptr, " ",
                      &saveptr);

  if (strncmp (p_param, "ANA", 3) == 0)
    {

      p_param = strtok_r (NULL, " ", &saveptr);
      if (p_param != NULL)
        fsm->cmd.psc = atol (p_param);
      else
        goto error;

      p_param = strtok_r (NULL, " ", &saveptr);
      if (p_param != NULL)
        fsm->cmd.arr = atol (p_param);
      else
        goto error;

      p_param = strtok_r (NULL, " ", &saveptr);
      if (p_param != NULL)
        fsm->cmd.count = atol (p_param);
      else
        goto error;

      p_param = strtok_r (NULL, " ", &saveptr);
      if (p_param != NULL)
        fsm->cmd.filter = atoi (p_param);
      else
        goto error;

      p_param = strtok_r (NULL, " ", &saveptr);
      if (p_param != NULL)
        fsm->cmd.decim = atoi (p_param);
      else
        goto error;
    }
  else if (strncmp (p_param, "FIR", 3) == 0)
    {
      do
        {
          p_param = strtok_r (NULL, " ", &saveptr);
        }
      while (p_param != NULL);
      fsm->state.coeff_acquired = true;
      return;
    }
  else
    {
      goto error;
    }

  fsm->state.cmd_valid = true;
  return;

error:
  fsm->state.cmd_valid = false;
  return;
}

#endif
