from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.arm_controller_dmesg_response_200 import ArmControllerDmesgResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import UNSET, Response, Unset


def _get_kwargs(
    *,
    lines: int | Unset = 200,
) -> dict[str, Any]:

    params: dict[str, Any] = {}

    params["lines"] = lines

    params = {k: v for k, v in params.items() if v is not UNSET and v is not None}

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/arm/controller/dmesg",
        "params": params,
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ArmControllerDmesgResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ArmControllerDmesgResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ArmControllerDmesgResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    lines: int | Unset = 200,
) -> Response[ArmControllerDmesgResponse200 | ErrorEnvelope]:
    """Read the arm controller board's kernel log tail

     Read-only. `lines` is clamped into `1..=1000` rather than rejected. Not available over WebSocket.

    Args:
        lines (int | Unset):  Default: 200.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmControllerDmesgResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        lines=lines,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    lines: int | Unset = 200,
) -> ArmControllerDmesgResponse200 | ErrorEnvelope | None:
    """Read the arm controller board's kernel log tail

     Read-only. `lines` is clamped into `1..=1000` rather than rejected. Not available over WebSocket.

    Args:
        lines (int | Unset):  Default: 200.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmControllerDmesgResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        lines=lines,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    lines: int | Unset = 200,
) -> Response[ArmControllerDmesgResponse200 | ErrorEnvelope]:
    """Read the arm controller board's kernel log tail

     Read-only. `lines` is clamped into `1..=1000` rather than rejected. Not available over WebSocket.

    Args:
        lines (int | Unset):  Default: 200.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ArmControllerDmesgResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        lines=lines,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    lines: int | Unset = 200,
) -> ArmControllerDmesgResponse200 | ErrorEnvelope | None:
    """Read the arm controller board's kernel log tail

     Read-only. `lines` is clamped into `1..=1000` rather than rejected. Not available over WebSocket.

    Args:
        lines (int | Unset):  Default: 200.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ArmControllerDmesgResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            lines=lines,
        )
    ).parsed
