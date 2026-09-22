from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.error_envelope import ErrorEnvelope
from ...models.eye_cmd_request import EyeCmdRequest
from ...models.eyes_cmd_response_200 import EyesCmdResponse200
from ...types import Response


def _get_kwargs(
    *,
    body: EyeCmdRequest,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/eyes/cmd",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | EyesCmdResponse200 | None:
    if response.status_code == 200:
        response_200 = EyesCmdResponse200.from_dict(response.json())

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
) -> Response[ErrorEnvelope | EyesCmdResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: EyeCmdRequest,
) -> Response[ErrorEnvelope | EyesCmdResponse200]:
    """Run one LED effect on one or both eye strips

     `target` and the effect share one flat object: the effect is selected by `effect` and carries that
    effect's own fields.

    Args:
        body (EyeCmdRequest): The advertised request body of `POST /v1/eyes/cmd`.

            This is a *flattened, advertise-only* view of [`d1fw_core::EyeCommand`],
            whose `#[serde(flatten)]` of the tagged [`d1fw_core::EyeEffect`] enum
            emitted an `allOf` over a `oneOf` — the shape `openapi-python-client`
            silently drops (it removed the `EyeCommand` model and skipped the
            endpoint). The daemon still deserializes the wire body into `EyeCommand`;
            nothing deserializes into this struct. `target` and `effect` are required,
            every colour/timing field is optional and applies only to the effects that
            name it.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | EyesCmdResponse200]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    body: EyeCmdRequest,
) -> ErrorEnvelope | EyesCmdResponse200 | None:
    """Run one LED effect on one or both eye strips

     `target` and the effect share one flat object: the effect is selected by `effect` and carries that
    effect's own fields.

    Args:
        body (EyeCmdRequest): The advertised request body of `POST /v1/eyes/cmd`.

            This is a *flattened, advertise-only* view of [`d1fw_core::EyeCommand`],
            whose `#[serde(flatten)]` of the tagged [`d1fw_core::EyeEffect`] enum
            emitted an `allOf` over a `oneOf` — the shape `openapi-python-client`
            silently drops (it removed the `EyeCommand` model and skipped the
            endpoint). The daemon still deserializes the wire body into `EyeCommand`;
            nothing deserializes into this struct. `target` and `effect` are required,
            every colour/timing field is optional and applies only to the effects that
            name it.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | EyesCmdResponse200
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: EyeCmdRequest,
) -> Response[ErrorEnvelope | EyesCmdResponse200]:
    """Run one LED effect on one or both eye strips

     `target` and the effect share one flat object: the effect is selected by `effect` and carries that
    effect's own fields.

    Args:
        body (EyeCmdRequest): The advertised request body of `POST /v1/eyes/cmd`.

            This is a *flattened, advertise-only* view of [`d1fw_core::EyeCommand`],
            whose `#[serde(flatten)]` of the tagged [`d1fw_core::EyeEffect`] enum
            emitted an `allOf` over a `oneOf` — the shape `openapi-python-client`
            silently drops (it removed the `EyeCommand` model and skipped the
            endpoint). The daemon still deserializes the wire body into `EyeCommand`;
            nothing deserializes into this struct. `target` and `effect` are required,
            every colour/timing field is optional and applies only to the effects that
            name it.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | EyesCmdResponse200]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: EyeCmdRequest,
) -> ErrorEnvelope | EyesCmdResponse200 | None:
    """Run one LED effect on one or both eye strips

     `target` and the effect share one flat object: the effect is selected by `effect` and carries that
    effect's own fields.

    Args:
        body (EyeCmdRequest): The advertised request body of `POST /v1/eyes/cmd`.

            This is a *flattened, advertise-only* view of [`d1fw_core::EyeCommand`],
            whose `#[serde(flatten)]` of the tagged [`d1fw_core::EyeEffect`] enum
            emitted an `allOf` over a `oneOf` — the shape `openapi-python-client`
            silently drops (it removed the `EyeCommand` model and skipped the
            endpoint). The daemon still deserializes the wire body into `EyeCommand`;
            nothing deserializes into this struct. `target` and `effect` are required,
            every colour/timing field is optional and applies only to the effects that
            name it.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | EyesCmdResponse200
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
